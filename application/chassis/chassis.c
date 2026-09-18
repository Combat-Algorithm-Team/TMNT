/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用，负责接收 robot_cmd 的控制命令并根据命令进行运动学解算，得到输出
 *        注意底盘采用右手坐标系：平面视图下，底盘前方为 x 正方向，右侧为 y 正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "power_calc.h"
#include "tool.h"
#include "user_lib.h"
#include "comm.h"
#include "robot_test.h"
#include "chassis_acceleration_plan.h"

/* 根据 robot_def.h 中的宏自动计算参数 */
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define LF               0
#define RF               1
#define RB               2
#define LB               3
#define POWER_CALIB_RPM_TO_RAD_PER_SEC 0.104719755f

#if POWER_CALIBRATION_ENABLE && defined(POWER_CALIBRATION_TARGET_WHEEL) && defined(POWER_CALIBRATION_TARGET_STEER)
#error You can only define one power calibration target group.
#endif

#if POWER_CALIBRATION_EXCITATION_ENABLE && \
    (!POWER_CALIBRATION_ENABLE || (!defined(POWER_CALIBRATION_TARGET_WHEEL) && !defined(POWER_CALIBRATION_TARGET_STEER)))
#error POWER_CALIBRATION_EXCITATION_ENABLE requires POWER_CALIBRATION_TARGET_WHEEL or POWER_CALIBRATION_TARGET_STEER.
#endif

/* 底盘应用包含的模块和信息存储，底盘为单例模式，因此不需要单独结构体 */
static INS_Instance *chassis_IMU_data; // 底盘IMU数据
static Publisher_t *chassis_pub;                    // 用于发布底盘数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘控制命令
// #endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
extern comm_cmd_t comm_cmd_data;
extern comm_upload_t comm_upload_data;
extern referee_info_t referee_info;           // 裁判系统数据
SuperCapInstance *cap;                                              // 超级电容
// 驱动与转向电机实例
DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
DJIMotorInstance *steering_lf, *steering_rf, *steering_rb, *steering_lb;
steering_wheelset_t wheelset_lf, wheelset_rf, wheelset_rb, wheelset_lb;
static float wheel_torque_ff_lf, wheel_torque_ff_rf, wheel_torque_ff_rb, wheel_torque_ff_lb;
static float chassis_ax, chassis_ay, chassis_alpha_z;
chassis_speed_measure_t speed_measure;
volatile chassis_mass_identification_data_t chassis_mass_identification_data;
#if CHASSIS_MASS_IDENTIFICATION_ENABLE || CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
static uint32_t mass_identification_tick;
static float mass_identification_force_x;
static float mass_identification_force_y;
static float mass_identification_torque_z;
#endif
static float mass_identification_last_wz;
volatile uint32_t power_calib_sample_cnt;
volatile float power_calib_cap_power_w;
volatile float power_calib_x_abs_omega_sum;
volatile float power_calib_x_tau2_sum;
volatile float power_calib_effective_power;
static float chassis_tilt_deadband_deg = 4.0f;
static uint8_t last_navi_stamp;
static uint32_t last_navi_DWT_stamp;
static uint8_t navi_speed_valid;
static float navi_raw_vx, navi_raw_vy, navi_raw_yaw;
// 跟随模式底盘的pid
// 目前未严格约定单位，后续如有需要再统一规范
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 0.1f,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0, // 0.0,  // 0.07,  // 0
    .DeadBand      = 1.5f,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 10000,
    

};

/* 车体速度环：目标为期望速度，反馈为底盘正运动学测得的实际速度，输出为目标加速度。 */
static PIDInstance Chassis_Acceleration_X_PID = {
    .Kp            = 10.0f,
    .Ki            = 0.0f,
    .Kd            = 0.0f,
    .DeadBand      = 0.0f,
    .IntegralLimit = 300000.0f,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 300000.0f,
};
static PIDInstance Chassis_Acceleration_Y_PID = {
    .Kp            = 10.0f,
    .Ki            = 0.0f,
    .Kd            = 0.0f,
    .DeadBand      = 0.0f,
    .IntegralLimit = 300000.0f,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 300000.0f,
};
static PIDInstance Chassis_Acceleration_Z_PID = {
    .Kp            = 100.0f,
    .Ki            = 0.0f,
    .Kd            = 0.0f,
    .DeadBand      = 0.0f,
    .IntegralLimit = 1000.0f,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 1000.0f,
};

/* 私有函数计算的中间变量，设为静态以避免参数传递开销 */
static float chassis_vx, chassis_vy, chassis_vw; // 将云台坐标系速度投影到底盘坐标系
static ramp_t rotate_ramp;
float follow_variable_angular_speed;
static int8_t follow_variable_rotate_direction = 1;
static float offset_angle;
static float sin_theta, cos_theta;
static float chassis_speed_direction;
static volatile float chassis_offset_delay_s        = 0.4f;
static volatile float chassis_offset_ff_limit_deg   = 90.0f;
static volatile float chassis_offset_ff_min_speed   = 100.0f;
static volatile float chassis_offset_ff_full_speed  = CHASSIS_HIGH_SPEED * 2.0f;

#if CHASSIS_MASS_IDENTIFICATION_ENABLE || CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
static void ChassisMassIdentificationCommand(void)
{
#if CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
    static const float torque_sequence_nm[] = {0.0f, 15.0f, 0.0f, -15.0f, 0.0f,
                                               -15.0f, 0.0f, 15.0f, 0.0f};
#else
    static const float force_sequence_n[] = {0.0f, 50.0f, 0.0f, -50.0f, 0.0f,
                                             -50.0f, 0.0f, 50.0f, 0.0f};
#endif
    static const uint16_t duration_ms[] = {1000U, 600U, 600U, 600U, 1000U,
                                           600U, 600U, 600U, 1000U};
    const float speed_limit_mm_s = 1500.0f;
    uint32_t elapsed = mass_identification_tick;
    uint32_t phase = 0U;
    uint32_t phase_count = sizeof(duration_ms) / sizeof(duration_ms[0]);
    uint32_t cycle_ticks = 0U;
    uint32_t i;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        mass_identification_tick = 0U;
        mass_identification_force_x = 0.0f;
        mass_identification_force_y = 0.0f;
        mass_identification_torque_z = 0.0f;
        chassis_mass_identification_data.enable = 0U;
        chassis_mass_identification_data.phase = 0U;
        return;
    }

    for (i = 0U; i < phase_count; ++i) cycle_ticks += duration_ms[i];
    elapsed %= cycle_ticks;
    for (phase = 0U; phase < phase_count - 1U; ++phase) {
        if (elapsed < duration_ms[phase]) break;
        elapsed -= duration_ms[phase];
    }

    chassis_mass_identification_data.enable = 1U;
    chassis_mass_identification_data.phase = (uint8_t)phase;
#if CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
    mass_identification_force_x = 0.0f;
    mass_identification_force_y = 0.0f;
    mass_identification_torque_z = torque_sequence_nm[phase];
    chassis_cmd_recv.vx = 0.0f;
    chassis_cmd_recv.vy = 0.0f;
#else
    mass_identification_force_x = force_sequence_n[phase];
    mass_identification_force_y = 0.0f;
    if (fabsf(speed_measure.real_vx) > speed_limit_mm_s &&
        mass_identification_force_x * speed_measure.real_vx > 0.0f) {
        mass_identification_force_x = 0.0f;
    }

    /* 该速度仅用于让舵轮保持直行；轮向实际输入由车体力直接分配。 */
    chassis_cmd_recv.vx = (mass_identification_force_x < 0.0f ? -100.0f : 100.0f) *
                          SPEED_TO_DJI_MOTOR_APS;
    chassis_cmd_recv.vy = 0.0f;
#endif
    chassis_cmd_recv.wz = 0.0f;
    chassis_cmd_recv.chassis_mode = CHASSIS_NO_FOLLOW;
    mass_identification_tick++;
}
#else
static void ChassisMassIdentificationCommand(void) {}
#endif

#if CHASSIS_MASS_IDENTIFICATION_ENABLE || CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
static void ChassisMassIdentificationUpdate(void)
{
    float ax = 0.0f, ay = 0.0f, fx = 0.0f, fy = 0.0f;
    float mz = 0.0f;
    DJIMotorInstance *motors[4] = {motor_lf, motor_rf, motor_rb, motor_lb};
    const steering_wheelset_t *wheels[4] = {&wheelset_lf, &wheelset_rf, &wheelset_rb, &wheelset_lb};
    int i;
    if (chassis_IMU_data != NULL) {
        ax = chassis_IMU_data->INS_data.INS_accel[0];
        ay = chassis_IMU_data->INS_data.INS_accel[1];
    }
    chassis_mass_identification_data.actual_ax = ax;
    chassis_mass_identification_data.actual_ay = ay;
    chassis_mass_identification_data.actual_wz = speed_measure.real_wz;
    chassis_mass_identification_data.actual_alpha_z =
        (speed_measure.real_wz - mass_identification_last_wz) * 1000.0f;
    mass_identification_last_wz = speed_measure.real_wz;
    chassis_mass_identification_data.actual_vx = speed_measure.real_vx * 0.001f;
    chassis_mass_identification_data.actual_vy = speed_measure.real_vy * 0.001f;
#if CHASSIS_MASS_IDENTIFICATION_ENABLE && !CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
    chassis_mass_identification_data.target_vx = 0.0f;
    chassis_mass_identification_data.target_vy = 0.0f;
    chassis_mass_identification_data.target_force_x = mass_identification_force_x;
    chassis_mass_identification_data.target_force_y = mass_identification_force_y;
    chassis_mass_identification_data.target_ax = mass_identification_force_x / TOTAL_MASS_KG;
    chassis_mass_identification_data.target_ay = mass_identification_force_y / TOTAL_MASS_KG;
#else
    chassis_mass_identification_data.target_vx = 0.0f;
    chassis_mass_identification_data.target_vy = 0.0f;
    chassis_mass_identification_data.target_force_x = 0.0f;
    chassis_mass_identification_data.target_force_y = 0.0f;
    chassis_mass_identification_data.target_ax = 0.0f;
    chassis_mass_identification_data.target_ay = 0.0f;
#endif
#if CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
    chassis_mass_identification_data.target_torque_z = mass_identification_torque_z;
#else
    chassis_mass_identification_data.target_torque_z = 0.0f;
#endif
    for (i = 0; i < 4; ++i) {
        float force;
        if (motors[i] == NULL) continue;
        force = (float)motors[i]->measure.real_current * CHASSIS_WHEEL_TORQUE_NM_PER_RAW /
                (RADIUS_WHEEL * 0.001f);
        if (wheels[i]->reverse_flag) force = -force;
        fx += force * arm_cos_f32(wheels[i]->angle_speed);
        fy += force * (-arm_sin_f32(wheels[i]->angle_speed));
    }
    chassis_mass_identification_data.resultant_force_x = fx;
    chassis_mass_identification_data.resultant_force_y = fy;
    /* 偏航力矩由力臂项反算，使用当前轮向方向和底盘几何尺寸。 */
    for (i = 0; i < 4; ++i) {
        float ux = arm_cos_f32(wheels[i]->angle_speed);
        float uy = -arm_sin_f32(wheels[i]->angle_speed);
        float force;
        if (motors[i] == NULL) continue;
        force = (float)motors[i]->measure.real_current * CHASSIS_WHEEL_TORQUE_NM_PER_RAW /
                (RADIUS_WHEEL * 0.001f);
        if (wheels[i]->reverse_flag) force = -force;
        mz += force * (ux * (i == LF || i == LB ? TRACK_WIDTH * 0.0005f : -TRACK_WIDTH * 0.0005f) -
                       uy * (i == LF || i == RF ? WHEEL_BASE * 0.0005f : -WHEEL_BASE * 0.0005f));
    }
    chassis_mass_identification_data.resultant_moment_z = mz;
}
#else
static void ChassisMassIdentificationUpdate(void) {}
#endif

/* 用于自旋变速策略的时间变量 */
// static float t;

void ChassisInit()
{
#ifdef CHASSIS_BOARD
    BMI088_Init_Config_s config = {
        .acc_int_config  = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_4},
        .gyro_int_config = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_5},
        .heat_pid_config = {
            .Kp            = 0.32f,
            .Ki            = 0.0004f,
            .Kd            = 0,
            .Improve       = PID_IMPROVE_NONE,
            .IntegralLimit = 0.90f,
            .MaxOut        = 0.95f,
        },
        .heat_pwm_config = {
            .htim      = &htim10,
            .channel   = TIM_CHANNEL_1,
            .dutyratio = 0,
            .period    = 5000 - 1,
        },
        .spi_acc_config = {
            .GPIOx      = GPIOA,
            .cs_pin     = GPIO_PIN_4,
            .spi_handle = &hspi1,
        },
        .spi_gyro_config = {
            .GPIOx      = GPIOB,
            .cs_pin     = GPIO_PIN_0,
            .spi_handle = &hspi1,
        },
        .cali_mode = BMI088_LOAD_PRE_CALI_MODE,
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,

    };
    chassis_IMU_data = INS_Init(BMI088Register(&config)); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // 四个轮子的参数一致，只需修改 tx_id 和反转标志位
    Motor_Init_Config_s chassis_motor_config = {
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 0.0, // 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
            },
            .current_feedforward_ptr = &wheel_torque_ff_lf
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .feedforward_flag     = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s steering_config = {
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 60,//12, // 0.24, // 0.31, // 0.45
                .Ki            = 7,
                .Kd            = 0,//0.02,//0.01,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement,
                .IntegralLimit = 20, 
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp            = 18,//6000,//10000, //11000,
                .Ki            = 8,    // 0
                .Kd            = 0.01,//5, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 3000,
                .MaxOut        = 20000 // 20000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020
    };
    steering_config.can_init_config.can_handle                              = &hcan2;
    steering_config.can_init_config.tx_id                                   = 1;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = LF;
    steering_lf                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan2;
    steering_config.can_init_config.tx_id                                   = 4;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = RF;
    steering_rf                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 3;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = RB;
    steering_rb                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 2;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = LB;
    steering_lb                                                             = DJIMotorInit(&steering_config);

    //  @todo: 当前未统一电机正反方向，仍需手动处理 reference 正负号，待电机模块支持后修复
    chassis_motor_config.can_init_config.can_handle                         = &hcan2;
    chassis_motor_config.can_init_config.tx_id                              = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = LF;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &wheel_torque_ff_lf;
    motor_lf                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan2;
    chassis_motor_config.can_init_config.tx_id                              = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = RF;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &wheel_torque_ff_rf;
    motor_rf                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = RB;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &wheel_torque_ff_rb;
    motor_rb                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = LB;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &wheel_torque_ff_lb;
    motor_lb                                                                = DJIMotorInit(&chassis_motor_config);

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0X180, // 超级电容默认接收id
            .rx_id      = 0x185, // 超级电容默认发送 ID，注意 tx/rx 对设备视角是相反的
        }};
    cap = SuperCapRegister(&cap_conf); // 超级电容初始化
    ramp_init(&rotate_ramp, 1000);
#endif
    // 发布订阅初始化；如果为双板则需要 CAN Comm 传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

static uint8_t ChassisIsRotateMode(void)
{
    return chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE ||
           chassis_cmd_recv.chassis_mode == CHASSIS_REVERSE_ROTATE;
}

static void ChassisOffsetAngleFeedforwardCalc(void)
{
    float offset_ff_deg = 0.0f;
    float move_speed;
    float speed_factor;

    if (ChassisIsRotateMode()) {
        move_speed = sqrtf(speed_measure.real_vx * speed_measure.real_vx + speed_measure.real_vy * speed_measure.real_vy);
        if (move_speed > chassis_offset_ff_min_speed) {
            if (chassis_offset_ff_full_speed > chassis_offset_ff_min_speed) {
                speed_factor = (move_speed - chassis_offset_ff_min_speed) /
                               (chassis_offset_ff_full_speed - chassis_offset_ff_min_speed);
                speed_factor = abs_limit(speed_factor, 1.0f);
            } else {
                speed_factor = 1.0f;
            }
        } else {
            speed_factor = 0.0f;
        }

        offset_ff_deg = speed_measure.real_wz * chassis_offset_delay_s * RAD_2_DEGREE * speed_factor;
        offset_ff_deg = abs_limit(offset_ff_deg, chassis_offset_ff_limit_deg);
    }
    comm_upload_data.debug_1 = offset_ff_deg;
    offset_angle += offset_ff_deg;
    cos_theta = arm_cos_f32(offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(offset_angle * DEGREE_2_RAD);
}

float cal_time_last = 0.0f;
float cal_time;

/**
 * @brief 计算经功率与摩擦约束后的车体线加速度和角加速度。
 * @param ax 输出 X 方向加速度(mm/s^2)。
 * @param ay 输出 Y 方向加速度(mm/s^2)。
 * @param alpha_z 输出 Z 轴角加速度(rad/s^2)。
 * @note PID 输出及原有控制量保持不变，约束求解在 C++ 单元中完成。
 */
static void ChassisAccelerationPlan(float *ax, float *ay, float *alpha_z)
{
    cal_time_last = DWT_GetTimeline_ms();
    ChassisAccelerationPlanInput input = {0};
    input.x_pid = &Chassis_Acceleration_X_PID;
    input.y_pid = &Chassis_Acceleration_Y_PID;
    input.z_pid = &Chassis_Acceleration_Z_PID;
    input.measured_velocity[0] = speed_measure.real_vx;
    input.measured_velocity[1] = speed_measure.real_vy;
    input.measured_velocity[2] = speed_measure.real_wz;
    input.target_velocity[0] = chassis_vx;
    input.target_velocity[1] = chassis_vy;
    input.target_velocity[2] = chassis_vw;

    input.wheel_angle_rad[LF] = wheelset_lf.angle_speed;
    input.wheel_angle_rad[RF] = wheelset_rf.angle_speed;
    input.wheel_angle_rad[RB] = wheelset_rb.angle_speed;
    input.wheel_angle_rad[LB] = wheelset_lb.angle_speed;
    input.wheel_reverse[LF] = wheelset_lf.reverse_flag;
    input.wheel_reverse[RF] = wheelset_rf.reverse_flag;
    input.wheel_reverse[RB] = wheelset_rb.reverse_flag;
    input.wheel_reverse[LB] = wheelset_lb.reverse_flag;

    input.wheel_speed_rpm[LF] = motor_lf != NULL ? motor_lf->measure.speed_rpm : 0.0f;
    input.wheel_speed_rpm[RF] = motor_rf != NULL ? motor_rf->measure.speed_rpm : 0.0f;
    input.wheel_speed_rpm[RB] = motor_rb != NULL ? motor_rb->measure.speed_rpm : 0.0f;
    input.wheel_speed_rpm[LB] = motor_lb != NULL ? motor_lb->measure.speed_rpm : 0.0f;
    input.wheel_base_current[LF] =
        motor_lf != NULL ? motor_lf->power_limit_output - wheel_torque_ff_lf : 0.0f;
    input.wheel_base_current[RF] =
        motor_rf != NULL ? motor_rf->power_limit_output - wheel_torque_ff_rf : 0.0f;
    input.wheel_base_current[RB] =
        motor_rb != NULL ? motor_rb->power_limit_output - wheel_torque_ff_rb : 0.0f;
    input.wheel_base_current[LB] =
        motor_lb != NULL ? motor_lb->power_limit_output - wheel_torque_ff_lb : 0.0f;
    input.power_limit_w = (float)chassis_cmd_recv.power_limit;

    ChassisAccelerationPlanQcp(&input, ax, ay, alpha_z);

    cal_time = DWT_GetTimeline_ms() - cal_time_last;
}

float force[4], ux[4], uy[4], moment_arm[4];

/**
 * @brief 将车体动力学需求分配为四轮力矩前馈。
 * @param 无；使用全局加速度、目标舵角和底盘几何参数。
 * @note 线加速度输入为 mm/s^2，内部换算为 m/s^2；输出为 DJI 原始电流指令。
 *       使用离线推导的直接分配式，不在控制周期内进行矩阵求逆。
 */
static void ChassisTorqueFeedforwardCalc(float target_force_x,
                                         float target_force_y,
                                         float target_torque_z)
{
    /* 刚体平面动力学：F = m*a，Mz = Iz*alpha；再将车体合力/力矩
     * 通过轮向单位向量构成的分配矩阵，以最小范数方式分配到四个轮。 */
    /* 底盘尺寸在 robot_def.h 中以 mm 定义；动力学力矩使用 N·m，
     * 因此这里直接将半轮距、半轴距换算为 m，保证 M = r × F 的单位一致。 */
    const float ht = TRACK_WIDTH * 0.0005f;
    const float hb = WHEEL_BASE * 0.0005f;
    const float *angles[4] = {&wheelset_lf.angle_speed, &wheelset_rf.angle_speed,
                              &wheelset_rb.angle_speed, &wheelset_lb.angle_speed};
    /* 加速度为 mm/s^2，乘 0.001 转为 m/s^2；力矩单位为 N*m。 */
    float wrench[3] = {target_force_x, target_force_y, target_torque_z};
    float *out[4] = {&wheel_torque_ff_lf, &wheel_torque_ff_rf, &wheel_torque_ff_rb, &wheel_torque_ff_lb};
    int s[4] = {1, -1, -1, 1};
    float yaw_norm = 0.0f;
    int i;

    for (i = 0; i < 4; ++i) {
        ux[i] = arm_cos_f32(*angles[i]);
        uy[i] = -arm_sin_f32(*angles[i]);
        moment_arm[i] = ux[i] * s[i] * ht - uy[i] * hb * (i < 2 ? 1.0f : -1.0f);
        yaw_norm += moment_arm[i] * moment_arm[i];
    }

    if (yaw_norm < 1.0e-6f) yaw_norm = 1.0f;
    for (i = 0; i < 4; ++i) {
        force[i] = 0.25f * (ux[i] * wrench[0] + uy[i] * wrench[1]) +
                   moment_arm[i] * wrench[2] / yaw_norm;
        if ((i == LF && wheelset_lf.reverse_flag) || (i == RF && wheelset_rf.reverse_flag) ||
            (i == RB && wheelset_rb.reverse_flag) || (i == LB && wheelset_lb.reverse_flag)) {
            force[i] = -force[i];
        }
        /* 轮端切向力乘轮半径得到 N*m，再换算为 DJI 原始电流指令。 */
        *out[i] = abs_limit(force[i] * RADIUS_WHEEL * 0.001f * CHASSIS_WHEEL_TORQUE_TO_CURRENT,
                            CHASSIS_TORQUE_FF_MAX_NM * CHASSIS_WHEEL_TORQUE_TO_CURRENT);
        if (!CHASSIS_TORQUE_FF_ENABLE)
            *out[i] = 0.0f;
#if !CHASSIS_MASS_IDENTIFICATION_ENABLE && !CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
        else if (fabsf(force[i] * RADIUS_WHEEL * 0.001f) < CHASSIS_TORQUE_FF_ZERO_DEADBAND_NM)
            *out[i] = 0.0f;
#endif
    }
}

/* 单个舵轮的逆运动学。rx/ry 是轮心相对底盘中心的坐标（x 向前、y 向右）。 */
static void SteeringCalculateWheel(steering_wheelset_t *wheel, DJIMotorInstance *steering,
                                   float rx, float ry, float encoder_zero)
{
    float target_angle_deg;

    /* 刚体平面运动：轮心速度 v_i = [vx - wz*ry, vy + wz*rx]。 */
    wheel->vx = chassis_vx - chassis_vw * ry;
    wheel->vy = chassis_vy + chassis_vw * rx;

    /* 舵机编码器角度沿用现有约定。舵角方向前的负号保持原坐标方向不变。 */
    wheel->angle_measure = ((float)steering->measure.ecd - encoder_zero) * ECD_ANGLE_COEF_DJI;
    wheel->angle_speed = -atan2f(wheel->vy, wheel->vx);
    target_angle_deg = wheel->angle_speed * RAD_2_DEGREE;

    /* 将舵角误差归一化到 [-180, 180]，避免跨圈时产生大幅转动。 */
    wheel->rotate_range = theta_format(target_angle_deg - wheel->angle_measure);

    /* 舵角超过 90° 时转 180°，同时反转轮速，轮子实际速度矢量保持不变。 */
    wheel->reverse_flag = (fabsf(wheel->rotate_range) > 90.0f) ? 1U : 0U;
    if (wheel->reverse_flag) {
        wheel->rotate_range += (wheel->rotate_range > 0.0f) ? -180.0f : 180.0f;
    }
    wheel->angle_target = wheel->angle_measure + wheel->rotate_range;
    wheel->angle_ref = steering->measure.total_angle + wheel->rotate_range;

    /* 轮速取轮心速度在当前舵角方向上的投影，再换算为电机转速单位。 */
    wheel->angle_diff = target_angle_deg - wheel->angle_measure;
    wheel->vt = arm_cos_f32(DEGREE_2_RAD * wheel->angle_diff) *
                sqrtf(wheel->vx * wheel->vx + wheel->vy * wheel->vy) * SPEED_TO_DJI_MOTOR_APS;
}

/**
 * @brief 执行底盘逆运动学，生成四轮舵角和轮向速度目标。
 * @param 无；使用全局车体速度及舵向反馈。
 * @note 舵角超过 90 度时翻转轮速方向，减少舵向电机的大角度动作。
 */
static void SteeringCalculate(void)
{
    const float half_track = TRACK_WIDTH * 0.5f;
    const float half_base = WHEEL_BASE * 0.5f;

    /* 四个轮心坐标对应原有 LF、RF、RB、LB 的速度正负号。 */
    SteeringCalculateWheel(&wheelset_lf, steering_lf, -half_base, -half_track, STEERING_LF_ECD);
    SteeringCalculateWheel(&wheelset_rf, steering_rf, -half_base, +half_track, STEERING_RF_ECD);
    SteeringCalculateWheel(&wheelset_rb, steering_rb, +half_base, +half_track, STEERING_RB_ECD);
    SteeringCalculateWheel(&wheelset_lb, steering_lb, +half_base, -half_track, STEERING_LB_ECD);
}

static uint8_t supercap_task_div_cnt = 0;

#if POWER_CALIBRATION_ENABLE
static void PowerCalibrationUpdate(void);
#endif

#if POWER_CALIBRATION_EXCITATION_ENABLE && POWER_CALIBRATION_ENABLE && defined(POWER_CALIBRATION_TARGET_WHEEL)
#define POWER_CALIB_WHEEL_EXCITATION_STEP_MS 2500U
#define POWER_CALIB_WHEEL_EXCITATION_STEP_COUNT 12U

static const float power_calib_wheel_excitation_speed_mm_s[POWER_CALIB_WHEEL_EXCITATION_STEP_COUNT] = {
    0.0f, 800.0f, -800.0f, 1600.0f, -1600.0f, 2400.0f,
    -2400.0f, 3200.0f, -3200.0f, 4000.0f, -4000.0f, 0.0f
};

static uint32_t power_calib_wheel_excitation_tick;

static void PowerCalibrationWheelExcitationReset(void)
{
    power_calib_wheel_excitation_tick = 0U;
}

static void PowerCalibrationWheelExcitationTask(void)
{
    uint32_t step_index;
    float ref;

    if (motor_lf == NULL || motor_rf == NULL || motor_rb == NULL || motor_lb == NULL ||
        steering_lf == NULL || steering_rf == NULL || steering_rb == NULL || steering_lb == NULL ||
        cap == NULL) {
        return;
    }

    DJIMotorStop(steering_lf);
    DJIMotorStop(steering_rf);
    DJIMotorStop(steering_rb);
    DJIMotorStop(steering_lb);

    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_rb);
    DJIMotorEnable(motor_lb);

    step_index = (power_calib_wheel_excitation_tick / POWER_CALIB_WHEEL_EXCITATION_STEP_MS) %
                 POWER_CALIB_WHEEL_EXCITATION_STEP_COUNT;
    ref = power_calib_wheel_excitation_speed_mm_s[step_index] * SPEED_TO_DJI_MOTOR_APS;

    DJIMotorSetRef(motor_lf, ref);
    DJIMotorSetRef(motor_rf, -ref);
    DJIMotorSetRef(motor_rb, ref);
    DJIMotorSetRef(motor_lb, -ref);

    cap->tx_data.enable_flag = SUPERCAP_DISABLE;
    cap->tx_data.charge_flag = 1;
    cap->tx_data.power_limit = referee_info.GameRobotStatus.chassis_power_limit;
    if (++supercap_task_div_cnt >= 5) {
        supercap_task_div_cnt = 0;
        SuperCapTask();
    }
    PowerCalibrationUpdate();

    power_calib_wheel_excitation_tick++;
}
#endif

#if POWER_CALIBRATION_EXCITATION_ENABLE && POWER_CALIBRATION_ENABLE && defined(POWER_CALIBRATION_TARGET_STEER)
#define POWER_CALIB_EXCITATION_STEP_MS 300U
#define POWER_CALIB_EXCITATION_STEP_COUNT 10U

static const float power_calib_excitation_steps[POWER_CALIB_EXCITATION_STEP_COUNT] = {
    0.0f, 8.0f, -8.0f, 18.0f, -18.0f, 35.0f, -35.0f, 50.0f, -50.0f, 0.0f
};

static uint8_t power_calib_excitation_initialized;
static uint32_t power_calib_excitation_tick;
static float power_calib_excitation_center_lf;
static float power_calib_excitation_center_rf;
static float power_calib_excitation_center_rb;
static float power_calib_excitation_center_lb;

static void PowerCalibrationExcitationReset(void)
{
    power_calib_excitation_initialized = 0U;
    power_calib_excitation_tick = 0U;
}

static void PowerCalibrationExcitationTask(void)
{
    uint32_t base_index;
    uint32_t lf_index, rf_index, rb_index, lb_index;

    if (motor_lf == NULL || motor_rf == NULL || motor_rb == NULL || motor_lb == NULL ||
        steering_lf == NULL || steering_rf == NULL || steering_rb == NULL || steering_lb == NULL ||
        cap == NULL) {
        return;
    }

    if (!power_calib_excitation_initialized) {
        power_calib_excitation_center_lf = steering_lf->measure.total_angle;
        power_calib_excitation_center_rf = steering_rf->measure.total_angle;
        power_calib_excitation_center_rb = steering_rb->measure.total_angle;
        power_calib_excitation_center_lb = steering_lb->measure.total_angle;
        power_calib_excitation_tick = 0U;
        power_calib_excitation_initialized = 1U;
    }

    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);

    DJIMotorEnable(steering_lf);
    DJIMotorEnable(steering_rf);
    DJIMotorEnable(steering_rb);
    DJIMotorEnable(steering_lb);

    base_index = (power_calib_excitation_tick / POWER_CALIB_EXCITATION_STEP_MS) % POWER_CALIB_EXCITATION_STEP_COUNT;
    lf_index = base_index;
    rf_index = (base_index + 2U) % POWER_CALIB_EXCITATION_STEP_COUNT;
    rb_index = (base_index + 4U) % POWER_CALIB_EXCITATION_STEP_COUNT;
    lb_index = (base_index + 6U) % POWER_CALIB_EXCITATION_STEP_COUNT;

    DJIMotorSetRef(steering_lf, power_calib_excitation_center_lf + power_calib_excitation_steps[lf_index]);
    DJIMotorSetRef(steering_rf, power_calib_excitation_center_rf + power_calib_excitation_steps[rf_index]);
    DJIMotorSetRef(steering_rb, power_calib_excitation_center_rb + power_calib_excitation_steps[rb_index]);
    DJIMotorSetRef(steering_lb, power_calib_excitation_center_lb + power_calib_excitation_steps[lb_index]);

    cap->tx_data.enable_flag = SUPERCAP_DISABLE;
    cap->tx_data.charge_flag = 1;
    cap->tx_data.power_limit = referee_info.GameRobotStatus.chassis_power_limit;
    if (++supercap_task_div_cnt >= 5) {
        supercap_task_div_cnt = 0;
        SuperCapTask();
    }
    PowerCalibrationUpdate();

    power_calib_excitation_tick++;
}
#endif

#if POWER_CALIBRATION_ENABLE
static void PowerCalibrationAccumulateMotor(DJIMotorInstance *motor,
                                            const PowerLimitator_Params_s *params,
                                            float speed_ratio,
                                            float *x_abs_omega_sum,
                                            float *x_tau2_sum,
                                            float *effective_power)
{
    float omega;
    float torque;

    if (motor == NULL || params == NULL) {
        return;
    }

    omega = motor->measure.speed_rpm * speed_ratio * POWER_CALIB_RPM_TO_RAD_PER_SEC;
    torque = (float)motor->measure.real_current * params->k0;

    *x_abs_omega_sum += fabsf(omega);
    *x_tau2_sum += torque * torque;
    *effective_power += torque * omega;
}

static void PowerCalibrationUpdate(void)
{
    float x_abs_omega_sum = 0.0f;
    float x_tau2_sum = 0.0f;
    float effective_power = 0.0f;

#if defined(POWER_CALIBRATION_TARGET_WHEEL)
    PowerCalibrationAccumulateMotor(motor_lf, &POWER_LIMITATOR_WHEEL_3508_DEFAULT, 1.0f / REDUCTION_RATIO_WHEEL,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
    PowerCalibrationAccumulateMotor(motor_rf, &POWER_LIMITATOR_WHEEL_3508_DEFAULT, 1.0f / REDUCTION_RATIO_WHEEL,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
    PowerCalibrationAccumulateMotor(motor_rb, &POWER_LIMITATOR_WHEEL_3508_DEFAULT, 1.0f / REDUCTION_RATIO_WHEEL,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
    PowerCalibrationAccumulateMotor(motor_lb, &POWER_LIMITATOR_WHEEL_3508_DEFAULT, 1.0f / REDUCTION_RATIO_WHEEL,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
#elif defined(POWER_CALIBRATION_TARGET_STEER)
    PowerCalibrationAccumulateMotor(steering_lf, &POWER_LIMITATOR_STEER_6020_DEFAULT, 1.0f,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
    PowerCalibrationAccumulateMotor(steering_rf, &POWER_LIMITATOR_STEER_6020_DEFAULT, 1.0f,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
    PowerCalibrationAccumulateMotor(steering_rb, &POWER_LIMITATOR_STEER_6020_DEFAULT, 1.0f,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
    PowerCalibrationAccumulateMotor(steering_lb, &POWER_LIMITATOR_STEER_6020_DEFAULT, 1.0f,
                                    &x_abs_omega_sum, &x_tau2_sum, &effective_power);
#else
#error POWER_CALIBRATION_ENABLE requires POWER_CALIBRATION_TARGET_WHEEL or POWER_CALIBRATION_TARGET_STEER.
#endif

    power_calib_cap_power_w = cap != NULL ? cap->chassis_real_power : 0.0f;
    power_calib_x_abs_omega_sum = x_abs_omega_sum;
    power_calib_x_tau2_sum = x_tau2_sum;
    power_calib_effective_power = effective_power;
    power_calib_sample_cnt++;
}
#endif

/**
 * @brief 超电控制算法
 *
 *
 */
 void Super_Cap_control()
 {
//     // 状态机逻辑,滞回
//     switch (SuperCap_state) {
//         case SUPER_STATE_LOW:
//             if (cap->cap_msg_s.CapVot > SUPER_VOLTAGE_THRESHOLD_HIGH) {
//                 SuperCap_state = SUPER_STATE_HIGH;
//             }
//             break;
//         case SUPER_STATE_HIGH:
//             if (cap->cap_msg_s.CapVot < SUPER_VOLTAGE_THRESHOLD_LOW) {
//                 SuperCap_state = SUPER_STATE_LOW;
//             }
//             break;
//         default:
//             SuperCap_state = SUPER_STATE_LOW;
//             break;
//     }

//     // 小于12V关闭
//     if (SuperCap_state == SUPER_STATE_LOW) {
//         Super_Voltage_Allow_Flag = SUPER_VOLTAGE_CLOSE;
//     } else if (SuperCap_state == SUPER_STATE_HIGH) {
//         Super_Voltage_Allow_Flag = SUPER_VOLTAGE_OPEN;
//     } else {
//         // none
//     }

//     // 用户允许开启超级电容，且电压充足
//     if (chassis_cmd_recv.SuperCap_flag_from_user == SUPER_USER_OPEN) {
//         cap->cap_msg_g.enabled = SUPER_CMD_OPEN;
//     } else {
//         cap->cap_msg_g.enabled = SUPER_CMD_CLOSE;
//     }
    cap->tx_data.enable_flag = SUPERCAP_ENABLE;
    cap->tx_data.charge_flag = 1;
    cap->tx_data.power_limit = chassis_cmd_recv.power_limit;
    if (++supercap_task_div_cnt >= 5) {
        supercap_task_div_cnt = 0;
        SuperCapTask();
    }
#if POWER_CALIBRATION_ENABLE
    PowerCalibrationUpdate();
#endif
     // 设定速度参考值
     DJIMotorSetRef(motor_lf, wheelset_lf.vt);
     DJIMotorSetRef(motor_rf, wheelset_rf.vt);
     DJIMotorSetRef(motor_lb, wheelset_lb.vt);
     DJIMotorSetRef(motor_rb, wheelset_rb.vt);
 }


/**
 * @brief 底盘速度方向映射与死区处理
 *
 */
static void ChassisSpeedMap()
{
    const float speed_deadband  = 10.0f / SPEED_TO_DJI_MOTOR_APS;
    const float rotate_deadband = 10.0f / (CHASSIS_R * SPEED_TO_DJI_MOTOR_APS);

    // 根据云台与底盘的角度 offset 将控制量映射到底盘坐标系
    // 辨识期间输入已经定义在底盘坐标系，必须跳过云台坐标变换。
#if CHASSIS_MASS_IDENTIFICATION_ENABLE || CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
#if CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
    chassis_vx = 0.0f;
    chassis_vy = 0.0f;
    chassis_vw = 0.8f;
#else
    chassis_vx = chassis_cmd_recv.vx;
    chassis_vy = 0.0f;
#endif
#else
    // 底盘逆时针为角度正方向；云台指向作为 y 轴
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
#endif

    if (fabsf(chassis_vx) < speed_deadband && fabsf(chassis_vy) < speed_deadband && fabsf(chassis_vw) < rotate_deadband)
    {
        chassis_vx = 0;
        chassis_vy = 0;
        chassis_vw = 1.0f / (CHASSIS_R * SPEED_TO_DJI_MOTOR_APS);
    }
}

/**
 * @brief 根据四轮实际轮速和舵角执行底盘正运动学解算。
 * @param 无；从四个轮向电机和舵向电机读取反馈。
 * @note 采用四轮速度分量的显式平均公式，直接发布当前解算结果。
 */
static void ChassisSpeedMeasure()
{
    /* 正运动学：将实际轮速投影到车体 x/y 方向，
     * 通过四轮平均值和两组偏航速度差显式解算 vx、vy、wz。 */
    const float half_track = TRACK_WIDTH * 0.5f;
    const float half_base = WHEEL_BASE * 0.5f;
    float lf_speed, rf_speed, rb_speed, lb_speed;
    float lf_angle, rf_angle, rb_angle, lb_angle;
    float raw_vx, raw_vy, raw_wz;
    float lf_vx, rf_vx, rb_vx, lb_vx;
    float lf_vy, rf_vy, rb_vy, lb_vy;

    if (motor_lf == NULL || motor_rf == NULL || motor_rb == NULL || motor_lb == NULL ||
        steering_lf == NULL || steering_rf == NULL || steering_rb == NULL || steering_lb == NULL) {
        speed_measure.real_vx = 0.0f;
        speed_measure.real_vy = 0.0f;
        speed_measure.real_wz = 0.0f;
        chassis_feedback_data.real_vx = 0.0f;
        chassis_feedback_data.real_vy = 0.0f;
        chassis_feedback_data.real_wz = 0.0f;
        return;
    }

    lf_speed = motor_lf->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;
    rf_speed = motor_rf->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;
    rb_speed = motor_rb->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;
    lb_speed = motor_lb->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;

    lf_angle = -((float)steering_lf->measure.ecd - STEERING_LF_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;
    rf_angle = -((float)steering_rf->measure.ecd - STEERING_RF_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;
    rb_angle = -((float)steering_rb->measure.ecd - STEERING_RB_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;
    lb_angle = -((float)steering_lb->measure.ecd - STEERING_LB_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;

    lf_vx = lf_speed * arm_cos_f32(lf_angle);
    rf_vx = rf_speed * arm_cos_f32(rf_angle);
    rb_vx = rb_speed * arm_cos_f32(rb_angle);
    lb_vx = lb_speed * arm_cos_f32(lb_angle);
    lf_vy = lf_speed * arm_sin_f32(lf_angle);
    rf_vy = rf_speed * arm_sin_f32(rf_angle);
    rb_vy = rb_speed * arm_sin_f32(rb_angle);
    lb_vy = lb_speed * arm_sin_f32(lb_angle);

    raw_vx = (lf_vx + rf_vx + rb_vx + lb_vx) * 0.25f;
    raw_vy = (lf_vy + rf_vy + rb_vy + lb_vy) * 0.25f;
    raw_wz = ((lf_vx - rf_vx - rb_vx + lb_vx) / (4.0f * half_track) +
              (-lf_vy - rf_vy + rb_vy + lb_vy) / (4.0f * half_base)) * 0.5f;

    speed_measure.real_vx = raw_vx;
    speed_measure.real_vy = raw_vy;
    speed_measure.real_wz = raw_wz;

    chassis_feedback_data.real_vx = speed_measure.real_vx;
    chassis_feedback_data.real_vy = speed_measure.real_vy;
    chassis_feedback_data.real_wz = speed_measure.real_wz;

}

/**
 * @brief 底盘速度插值
 *
 */
static void ChassisSpeedInterpolate()
{
    uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
    float delta_yaw;
    float sin_delta_yaw, cos_delta_yaw;

    if (comm_cmd_data.navi_stamp == 0 || chassis_IMU_data == NULL) {
        last_navi_stamp = comm_cmd_data.navi_stamp;
        last_navi_DWT_stamp = now_ms;
        navi_speed_valid = 0;
        return;
    }

    if (!navi_speed_valid || last_navi_stamp != comm_cmd_data.navi_stamp)
    {
        navi_raw_vx = chassis_vx;
        navi_raw_vy = chassis_vy;
        navi_raw_yaw = chassis_IMU_data->output.Yaw_total_angle;
        last_navi_DWT_stamp = now_ms;
        last_navi_stamp = comm_cmd_data.navi_stamp;
        navi_speed_valid = 1;
        return;
    }

    if (now_ms - last_navi_DWT_stamp > 1000U) {
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        return;
    }

    delta_yaw = navi_raw_yaw - chassis_IMU_data->output.Yaw_total_angle;
    sin_delta_yaw = arm_sin_f32(delta_yaw);
    cos_delta_yaw = arm_cos_f32(delta_yaw);
    chassis_vx = navi_raw_vx * cos_delta_yaw - navi_raw_vy * sin_delta_yaw;
    chassis_vy = navi_raw_vx * sin_delta_yaw + navi_raw_vy * cos_delta_yaw;
}

/**
 * @brief 底盘梯度方向
 *
 */
static void ChassisTiltCalc()
{
    float pitch, roll;
    float sin_pitch, sin_roll, cos_pitch, cos_roll;
    float tilt_x, tilt_y;
    float tilt_alpha;
    float tilt_direction;

    if (chassis_IMU_data == NULL) {
        chassis_feedback_data.tilt_direction = 0.0f;
        chassis_feedback_data.tilt_angle = 0.0f;
        return;
    }

    pitch     = chassis_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    roll      = chassis_IMU_data->output.INS_angle[INS_ROLL_ADDRESS_OFFSET];
    sin_pitch = arm_sin_f32(pitch);
    sin_roll  = arm_sin_f32(roll);
    cos_pitch = arm_cos_f32(pitch);
    cos_roll  = arm_cos_f32(roll);
    tilt_x    = -sin_pitch;
    tilt_y    = sin_roll * cos_pitch;

    tilt_alpha = atan2f(sqrtf(tilt_x * tilt_x + tilt_y * tilt_y), cos_roll * cos_pitch) * RAD_2_DEGREE;
    if (tilt_alpha < chassis_tilt_deadband_deg) {
        chassis_feedback_data.tilt_direction = 0.0f;
        chassis_feedback_data.tilt_angle = 0.0f;
        return;
    }

    tilt_direction = atan2f(tilt_y, tilt_x) * RAD_2_DEGREE;
    if (tilt_direction < 0.0f) {
        tilt_direction += 360.0f;
    }

    chassis_feedback_data.tilt_direction    = tilt_direction;
    chassis_feedback_data.tilt_angle        = tilt_alpha;
}

/**
 * @brief 底盘速度方向
 *
 */
static void GetChassisSpeedDirection()
{
    const float speed_deadband = 10.0f / SPEED_TO_DJI_MOTOR_APS;
    float chassis_speed;
    float speed_direction;

    chassis_speed = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
    if (chassis_speed < speed_deadband) {
        return;
    }

    speed_direction = atan2f(chassis_vy, chassis_vx) * RAD_2_DEGREE;
    speed_direction = theta_format(speed_direction);

    while (speed_direction > 90.0f) {
        speed_direction -= 180.0f;
    }
    while (speed_direction < -90.0f) {
        speed_direction += 180.0f;
    }

    chassis_speed_direction = speed_direction;
}

/**
 * @brief 跨越地形时速度控制
 *
 */
static void TerrainSpeedControl()
{
    float chassis_speed;
    float scale;

    if(!comm_cmd_data.terrain_state)
        return;
    
    chassis_speed = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);

    if(comm_cmd_data.terrain_state == TERRAIN_BUMP)
    {
        chassis_vx *= 2;
        chassis_vy *= 2;
    }
    if(comm_cmd_data.terrain_state == TERRAIN_FORTRESS)
    {
        if (chassis_speed <= 500.0f) {
            return;
        }

        scale = 500.0f / chassis_speed;
        chassis_vx *= scale;
        chassis_vy *= scale;
    }
}

/**
 * @brief 堡垒上坡助力
 *
 */
static void FortressAssist()
{
    if(comm_cmd_data.terrain_state != TERRAIN_FORTRESS)
        return;

    const float speed_deadband = 10.0f / SPEED_TO_DJI_MOTOR_APS;
    const float tilt_angle_cap = 25.0f;
    float chassis_speed;
    float speed_direction;
    float delta_direction;
    float dir_factor;
    float tilt_factor;
    float assist_scale;

    chassis_speed = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
    if (chassis_speed < speed_deadband) {
        return;
    }

    if (chassis_feedback_data.tilt_angle <= 0.0f) {
        return;
    }

    speed_direction = theta_format(-atan2f(chassis_vy, chassis_vx) * RAD_2_DEGREE);
    delta_direction = speed_direction - chassis_feedback_data.tilt_direction;

    while (delta_direction > 180.0f) {
        delta_direction -= 360.0f;
    }
    while (delta_direction < -180.0f) {
        delta_direction += 360.0f;
    }

    dir_factor = arm_cos_f32(delta_direction * DEGREE_2_RAD);
    if (dir_factor <= 0.0f) {
        return;
    }

    tilt_factor = chassis_feedback_data.tilt_angle / tilt_angle_cap;
    tilt_factor = abs_limit(tilt_factor, 1.0f);
    if (tilt_factor <= 0.0f) {
        return;
    }

    assist_scale = 1.0f + 0.5f * dir_factor * tilt_factor;
    chassis_vx *= assist_scale;
    chassis_vy *= assist_scale;
}

/* 机器人底盘控制核心任务 */
/**
 * @brief 底盘 1 kHz 控制任务入口。
 * @param 无；控制命令通过 chassis_cmd 主题接收。
 * @note 正常流程依次执行正解算、速度映射、加速度规划、力矩分配和电机目标更新；
 *       零力矩模式仍会停止轮向与舵向电机。
 */
void ChassisTask()
{
    // 获取新的控制信息

    SubGetMessage(chassis_sub, &chassis_cmd_recv);

#ifdef CHASSIS_BOARD
    chassis_cmd_recv.power_limit = 10.0f;
    /* 功率约束已前移至 ChassisAccelerationPlan，关闭电机层二次缩放。 */
    DJIMotorPowerControlEnable(0U);
#endif

    ChassisMassIdentificationCommand();
    
#ifdef CHASSIS_BOARD
    ChassisSpeedMeasure();
    ChassisTiltCalc();
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现关键模块离线或遥控器急停，则关闭电机输出
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorStop(steering_lf);
        DJIMotorStop(steering_rf);
        DJIMotorStop(steering_rb);
        DJIMotorStop(steering_lb);
#if POWER_CALIBRATION_EXCITATION_ENABLE && POWER_CALIBRATION_ENABLE && defined(POWER_CALIBRATION_TARGET_STEER)
        PowerCalibrationExcitationReset();
#endif
#if POWER_CALIBRATION_EXCITATION_ENABLE && POWER_CALIBRATION_ENABLE && defined(POWER_CALIBRATION_TARGET_WHEEL)
        PowerCalibrationWheelExcitationReset();
#endif
    } else { // 正常工作
#if POWER_CALIBRATION_EXCITATION_ENABLE && POWER_CALIBRATION_ENABLE && defined(POWER_CALIBRATION_TARGET_STEER)
        PowerCalibrationExcitationTask();
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
        return;
#endif
#if POWER_CALIBRATION_EXCITATION_ENABLE && POWER_CALIBRATION_ENABLE && defined(POWER_CALIBRATION_TARGET_WHEEL)
        PowerCalibrationWheelExcitationTask();
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
        return;
#endif
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        DJIMotorEnable(steering_lf);
        DJIMotorEnable(steering_rf);
        DJIMotorEnable(steering_rb);
        DJIMotorEnable(steering_lb);
    }

    static float current_speed_vw, vw_set;
    // static ramp_t rotate_ramp;

    offset_angle       = chassis_cmd_recv.offset_angle;// + chassis_cmd_recv.gimbal_error_angle;
    // offset_angle_watch = offset_angle;
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            // 底盘不自旋，但保持全向机动，一般用于调整云台姿态
            // 当前模式下保留外部 wz 指令（不强制置零）
            //chassis_cmd_recv.wz = 0;
            chassis_vw = 0;
            follow_variable_angular_speed = 0.0f;
            follow_variable_rotate_direction = 1;
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台

            //  if (chassis_cmd_recv.offset_angle <= 90 && chassis_cmd_recv.offset_angle >= -90) // 0附近
            //     offset_angle = -chassis_cmd_recv.offset_angle;
            //  else {
            //      offset_angle = -(chassis_cmd_recv.offset_angle >= 0 ? chassis_cmd_recv.offset_angle - 180 : chassis_cmd_recv.offset_angle + 180);
            //  }
            /* Temporary test: follow mode outputs a triangular-wave gyro speed. */
            follow_variable_angular_speed +=
                (float)follow_variable_rotate_direction *
                CHASSIS_FOLLOW_VARIABLE_ROTATE_ANGULAR_ACCELERATION_RAD_S2 * 0.001f;
            if (follow_variable_angular_speed >= CHASSIS_FOLLOW_VARIABLE_ROTATE_ANGULAR_SPEED_MAX_RAD_S) {
                follow_variable_angular_speed = CHASSIS_FOLLOW_VARIABLE_ROTATE_ANGULAR_SPEED_MAX_RAD_S;
                follow_variable_rotate_direction = -1;
            } else if (follow_variable_angular_speed <= CHASSIS_FOLLOW_VARIABLE_ROTATE_ANGULAR_SPEED_MIN_RAD_S) {
                follow_variable_angular_speed = CHASSIS_FOLLOW_VARIABLE_ROTATE_ANGULAR_SPEED_MIN_RAD_S;
                follow_variable_rotate_direction = 1;
            }
            chassis_vw = follow_variable_angular_speed;
            chassis_cmd_recv.vx *= 0.6f;
            chassis_cmd_recv.vy *= 0.6f;


            ramp_init(&rotate_ramp, 250);
            break;
            // 旋转模式调参说明
        case CHASSIS_ROTATE: // 自旋，同时保持全向机动；当前 wz 保持定速，后续可增加不规则变速策略
           //  if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_PMOS_OPEN) {
           //      vw_set = 7000;
           //  } else {
                vw_set = 10 * ((float)chassis_cmd_recv.chassis_rotate_speed / 255.0f);
            // }


            chassis_vw       = (current_speed_vw + (vw_set - current_speed_vw) * ramp_calc(&rotate_ramp));
            current_speed_vw = chassis_vw;
            follow_variable_angular_speed = 0.0f;
            follow_variable_rotate_direction = 1;
            

            // chassis_cmd_recv.wz = chassis_vw;
            chassis_cmd_recv.vx *= 0.6;
            chassis_cmd_recv.vy *= 0.6;
            break;
            
        case CHASSIS_REVERSE_ROTATE:
            // chassis_cmd_recv.wz = -2500;
            chassis_vw          = -2;
            offset_angle       += 22.0f;
            break;
        default:
            break;
    }
    ChassisOffsetAngleFeedforwardCalc();
    ChassisSpeedMap();
    GetChassisSpeedDirection();
    ChassisSpeedInterpolate();
    TerrainSpeedControl();
    FortressAssist();
    // 根据控制模式进行正运动学解算,计算底盘输出
    SteeringCalculate();
    /* 根据本周期轮向执行二次约束规划，再由动力学分配轮端力矩前馈。 */
    ChassisAccelerationPlan(&chassis_ax, &chassis_ay, &chassis_alpha_z);
#if CHASSIS_YAW_INERTIA_IDENTIFICATION_ENABLE
    ChassisTorqueFeedforwardCalc(0.0f, 0.0f, mass_identification_torque_z);
#elif CHASSIS_MASS_IDENTIFICATION_ENABLE
    ChassisTorqueFeedforwardCalc(mass_identification_force_x, mass_identification_force_y, 0.0f);
#else
    ChassisTorqueFeedforwardCalc(TOTAL_MASS_KG * chassis_ax * 0.001f,
                                 TOTAL_MASS_KG * chassis_ay * 0.001f,
                                 CHASSIS_YAW_INERTIA_KG_M2 * chassis_alpha_z);
#endif
    ChassisMassIdentificationUpdate();

    DJIMotorSetRef(steering_lf, wheelset_lf.angle_ref);
    DJIMotorSetRef(steering_rf, wheelset_rf.angle_ref);
    DJIMotorSetRef(steering_rb, wheelset_rb.angle_ref);
    DJIMotorSetRef(steering_lb, wheelset_lb.angle_ref);
    // DJIMotorSetRef(steering_lf, 0);
    // 根据裁判系统与电容反馈对输出限幅，并设定闭环参考
    Super_Cap_control();



    // 给电容发送数据
    //SuperCapSend(cap, (uint8_t *)&cap->cap_msg_g);

    // 推送反馈消息
    //memcpy(&chassis_feedback_data.CapFlag_open_from_real, &cap->cap_msg_s.SuperCap_open_flag_from_real, sizeof(uint8_t));
    //memcpy(&chassis_feedback_data.cap_voltage, &cap->cap_msg_s.CapVot, sizeof(float));
   // memcpy(&chassis_feedback_data.chassis_voltage, &cap->cap_msg_s.chassis_voltage_from_cap, sizeof(float));
#endif
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}

