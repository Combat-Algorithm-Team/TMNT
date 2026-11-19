#include "power_calc.h"

#include <math.h>

/* 单轮功率模型沿用转子 RPM 和 C620 电流原始值。 */
static const float speed_loss_coefficient = 4.88405e-07f;
static const float current_loss_coefficient = 1.749615052e-07f;
static const float mechanical_power_coefficient = 1.050994734e-7f;
static const float idle_power = 3.0f;
static const float wheel_current_limit = 15000.0f;

void WheelPowerLimitCurrents(float max_power,
                            const float motor_speed_rpm[WHEEL_POWER_MOTOR_COUNT],
                            float motor_current[WHEEL_POWER_MOTOR_COUNT],
                            const uint8_t active[WHEEL_POWER_MOTOR_COUNT])
{
    float wheel_power[WHEEL_POWER_MOTOR_COUNT] = {0.0f};
    float total_power = 0.0f;

    for (uint8_t i = 0; i < WHEEL_POWER_MOTOR_COUNT; ++i) {
        if (!active[i]) {
            continue;
        }

        float rpm = motor_speed_rpm[i];
        float current = motor_current[i];
        if (!(max_power > 0.0f) || !isfinite(rpm) || !isfinite(current)) {
            motor_current[i] = 0.0f;
            continue;
        }

        current = fminf(fmaxf(current, -wheel_current_limit), wheel_current_limit);
        motor_current[i] = current;
        wheel_power[i] = mechanical_power_coefficient * rpm * current +
                         speed_loss_coefficient * rpm * rpm +
                         current_loss_coefficient * current * current + idle_power;
        // 旧策略只汇总正功率，制动功率不抵扣用电预算。
        if (wheel_power[i] > 0.0f) {
            total_power += wheel_power[i];
        }
    }

    if (!(total_power > max_power)) {
        return;
    }

    const float power_scale = max_power / total_power;
    for (uint8_t i = 0; i < WHEEL_POWER_MOTOR_COUNT; ++i) {
        if (wheel_power[i] <= 0.0f || motor_current[i] == 0.0f) {
            continue;
        }

        float rpm = motor_speed_rpm[i];
        float current = motor_current[i];
        // 固定指令方向，以电流绝对值为未知数反解二次方程。
        float b = mechanical_power_coefficient * rpm * (current > 0.0f ? 1.0f : -1.0f);
        float c = speed_loss_coefficient * rpm * rpm + idle_power - wheel_power[i] * power_scale;
        float discriminant = b * b - 4.0f * current_loss_coefficient * c;
        float magnitude = 0.0f;

        // 预算低于模型可达功率时，卸掉电流；可解时也不能反向或放大指令。
        if (discriminant >= 0.0f) {
            magnitude = (-b + sqrtf(discriminant)) / (2.0f * current_loss_coefficient);
            magnitude = fminf(fmaxf(magnitude, 0.0f), fabsf(current));
        }
        motor_current[i] = current > 0.0f ? magnitude : -magnitude;
    }
}
