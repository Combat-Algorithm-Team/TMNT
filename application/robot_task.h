/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "referee_init.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "user_lib.h"

#include "bsp_log.h"
#include "led.h"
#include "buzzer.h"
#include "ins_task.h"
#include "referee_UI.h"

#include "robot_cmd.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "motor_task.h"
#include "omni_UI.h"

typedef struct
{
    volatile float current_ms;
    volatile float max_ms;
} RobotTaskTiming_s;

volatile RobotTaskTiming_s robot_task_ins_timing;
volatile RobotTaskTiming_s robot_task_cmd_timing;
volatile RobotTaskTiming_s robot_task_chassis_timing;
volatile RobotTaskTiming_s robot_task_gimbal_timing;
volatile RobotTaskTiming_s robot_task_shoot_timing;
volatile RobotTaskTiming_s robot_task_motor_timing;
volatile RobotTaskTiming_s robot_task_daemon_timing;

static void RobotTaskTimingBegin(uint32_t *cycle_count)
{
    (void)DWT_GetDeltaT(cycle_count);
}

static void RobotTaskTimingEnd(uint32_t *cycle_count, volatile RobotTaskTiming_s *timing)
{
    float elapsed_ms = 1000.0f * DWT_GetDeltaT(cycle_count);

    timing->current_ms = elapsed_ms;
    if (elapsed_ms > timing->max_ms) {
        timing->max_ms = elapsed_ms;
    }
}

//! 任务直接在cubeMX中配置,不再使用这种方式
__attribute__((noreturn)) void StartINSTASK(void *argument)
{
    UNUSED(argument);
    uint32_t cycle_count = 0U;
    LOGINFO("[freeRTOS] INS Task Start");
    while (1) {
        RobotTaskTimingBegin(&cycle_count);
        INS_Task();
        RobotTaskTimingEnd(&cycle_count, &robot_task_ins_timing);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    UNUSED(argument);
    uint32_t cycle_count = 0U;
    LOGINFO("[freeRTOS] RobotCMD Task Start");
    for (;;) {
        RobotTaskTimingBegin(&cycle_count);
        RobotCMDTask();
        RobotTaskTimingEnd(&cycle_count, &robot_task_cmd_timing);
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ChassisTask(void *argument)
{
    UNUSED(argument);
    uint32_t cycle_count = 0U;
    for (;;) {
        RobotTaskTimingBegin(&cycle_count);
        ChassisTask();
        RobotTaskTimingEnd(&cycle_count, &robot_task_chassis_timing);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _GimbalTask(void *argument)
{
    UNUSED(argument);
    uint32_t cycle_count = 0U;
    for (;;) {
        RobotTaskTimingBegin(&cycle_count);
        GimbalTask();
        RobotTaskTimingEnd(&cycle_count, &robot_task_gimbal_timing);
        osDelay(1);
    }
}
__attribute__((noreturn)) void _ShootTask(void *argument)
{
    UNUSED(argument);
    uint32_t cycle_count = 0U;
    for (;;) {
        RobotTaskTimingBegin(&cycle_count);
        ShootTask();
        RobotTaskTimingEnd(&cycle_count, &robot_task_shoot_timing);
        osDelay(1);
    }
}
__attribute__((noreturn)) void motorControlTask(void *argument)
{
    UNUSED(argument);
    uint32_t cycle_count = 0U;
    for (;;) {
        RobotTaskTimingBegin(&cycle_count);
        MotorControlTask();
        RobotTaskTimingEnd(&cycle_count, &robot_task_motor_timing);
        osDelay(1);
    }
}

// __attribute__((noreturn)) void _UITask(void *argument)
// {
//     // UI_Init();
//     // for (;;) {
//     //     UIDynamicRefresh();
//     //     osDelay(50);
//     // }
// }

__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    UNUSED(argument);
    uint32_t cycle_count = 0U;
    for (;;) {
        RobotTaskTimingBegin(&cycle_count);
        DaemonTask();
        RobotTaskTimingEnd(&cycle_count, &robot_task_daemon_timing);
        osDelay(1);
    }
}
