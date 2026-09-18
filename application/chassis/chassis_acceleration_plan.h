#ifndef CHASSIS_ACCELERATION_PLAN_H
#define CHASSIS_ACCELERATION_PLAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "controller.h"
#include <stdint.h>

#define CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT 4U

typedef struct
{
    PIDInstance *x_pid;
    PIDInstance *y_pid;
    PIDInstance *z_pid;
    float measured_velocity[3];
    float target_velocity[3];
    float wheel_angle_rad[CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT];
    float wheel_speed_rpm[CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT];
    float wheel_base_current[CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT];
    uint8_t wheel_reverse[CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT];
    float power_limit_w;
} ChassisAccelerationPlanInput;

void ChassisAccelerationPlanQcp(const ChassisAccelerationPlanInput *input,
                                float *ax,
                                float *ay,
                                float *alpha_z);

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_ACCELERATION_PLAN_H
