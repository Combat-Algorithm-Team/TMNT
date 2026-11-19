#pragma once

#include <stdint.h>

#define WHEEL_POWER_MOTOR_COUNT 2u

/**
 * @brief 两个轮电机共用同一个功率预算，原地限幅电流指令。
 * @param max_power 双轮共用的功率上限（W），非正值将活动轮的电流置零。
 * @param motor_speed_rpm 电机转子转速（RPM），不做减速比换算。
 * @param motor_current C620 原始电流指令，限幅前后使用相同单位。
 * @param active 仅对非零项对应的轮电机估算功率并改写电流。
 */
void WheelPowerLimitCurrents(float max_power,
                            const float motor_speed_rpm[WHEEL_POWER_MOTOR_COUNT],
                            float motor_current[WHEEL_POWER_MOTOR_COUNT],
                            const uint8_t active[WHEEL_POWER_MOTOR_COUNT]);
