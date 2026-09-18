#include "chassis_acceleration_plan.h"

#ifdef abs
#undef abs
#endif

#include <cmath>

#include "power_calc.h"
#include "qcp_solver.hpp"
#include "robot_def.h"

namespace {

using rmcs_core::controller::chassis::QcpSolver;

constexpr double kMillimeterToMeter = 1.0e-3;
constexpr double kRpmToRadPerSecond = 0.10471975511965977;

void CalculateQuadraticConstraint(const ChassisAccelerationPlanInput& input,
                                  const Eigen::Vector2d& translation_direction,
                                  double angular_direction,
                                  QcpSolver::QuadraticConstraint& constraint)
{
    const double half_track = TRACK_WIDTH * 0.0005;
    const double half_base = WHEEL_BASE * 0.0005;
    const double wheel_radius = RADIUS_WHEEL * kMillimeterToMeter;
    const double wheel_speed_ratio = kRpmToRadPerSecond / REDUCTION_RATIO_WHEEL;
    const PowerLimitator_Params_s& power = POWER_LIMITATOR_WHEEL_3508_DEFAULT;
    double yaw_norm = 0.0;
    double moment_arm[CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT];
    double ux[CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT];
    double uy[CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT];

    for (unsigned int i = 0; i < CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT; ++i) {
        const double lateral_sign = (i == 0U || i == 3U) ? 1.0 : -1.0;
        const double longitudinal_sign = i < 2U ? 1.0 : -1.0;
        ux[i] = std::cos(input.wheel_angle_rad[i]);
        uy[i] = -std::sin(input.wheel_angle_rad[i]);
        moment_arm[i] = ux[i] * lateral_sign * half_track
                      - uy[i] * longitudinal_sign * half_base;
        yaw_norm += moment_arm[i] * moment_arm[i];
    }

    if (yaw_norm < 1.0e-6)
        yaw_norm = 1.0;

    constraint = QcpSolver::QuadraticConstraint{0.0, 0.0, 0.0, 0.0, 0.0,
                                                 -input.power_limit_w};
    for (unsigned int i = 0; i < CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT; ++i) {
        const double reverse = input.wheel_reverse[i] ? -1.0 : 1.0;
        const double force_translation =
            0.25 * TOTAL_MASS_KG
            * (ux[i] * translation_direction.x() + uy[i] * translation_direction.y());
        const double force_rotation =
            moment_arm[i] * CHASSIS_YAW_INERTIA_KG_M2 * angular_direction / yaw_norm;
        const double torque_translation = reverse * wheel_radius * force_translation;
        const double torque_rotation = reverse * wheel_radius * force_rotation;
        const double torque_base = input.wheel_base_current[i] * power.k0;
        const double wheel_velocity = input.wheel_speed_rpm[i] * wheel_speed_ratio;
        const double linear_power_coefficient = 2.0 * power.k2 * torque_base + wheel_velocity;

        constraint.a += power.k2 * torque_translation * torque_translation;
        constraint.b += 2.0 * power.k2 * torque_translation * torque_rotation;
        constraint.c += power.k2 * torque_rotation * torque_rotation;
        constraint.d += linear_power_coefficient * torque_translation;
        constraint.e += linear_power_coefficient * torque_rotation;
        constraint.f += torque_base * (power.k2 * torque_base + wheel_velocity)
                      + power.k1 * std::abs(wheel_velocity)
                      + power.k3 / CHASSIS_ACCELERATION_PLAN_WHEEL_COUNT;
    }
}

} // namespace

extern "C" void ChassisAccelerationPlanQcp(const ChassisAccelerationPlanInput *input,
                                            float *ax,
                                            float *ay,
                                            float *alpha_z)
{
    if (input == NULL || ax == NULL || ay == NULL || alpha_z == NULL)
        return;

    *ax = PIDCalculate(input->x_pid, input->measured_velocity[0], input->target_velocity[0]);
    *ay = PIDCalculate(input->y_pid, input->measured_velocity[1], input->target_velocity[1]);
    *alpha_z = PIDCalculate(input->z_pid, input->measured_velocity[2], input->target_velocity[2]);

    if (input->power_limit_w <= 0.0f)
        return;

    Eigen::Vector2d translation_direction(*ax * kMillimeterToMeter,
                                          *ay * kMillimeterToMeter);
    double translation_max = translation_direction.norm();
    if (translation_max > 0.0)
        translation_direction /= translation_max;

    const double angular_direction = *alpha_z >= 0.0f ? 1.0 : -1.0;
    const double angular_max = std::abs(*alpha_z);
    if (translation_max <= 0.0 && angular_max <= 0.0)
        return;

    QcpSolver::QuadraticConstraint quadratic_constraint;
    CalculateQuadraticConstraint(*input, translation_direction, angular_direction,
                                 quadratic_constraint);

    const double friction_limit = CHASSIS_QCP_FRICTION_COEFFICIENT * 9.81;
    const double angular_friction_limit =
        friction_limit * TOTAL_MASS_KG * (CHASSIS_R * kMillimeterToMeter)
        / CHASSIS_YAW_INERTIA_KG_M2;
    const Eigen::Vector2d best_point = QcpSolver::solve(
        Eigen::Vector2d(CHASSIS_QCP_TRANSLATIONAL_WEIGHT, CHASSIS_QCP_ANGULAR_WEIGHT),
        QcpSolver::BoundaryConstraint{translation_max, angular_max},
        QcpSolver::RhombusConstraint{friction_limit, angular_friction_limit},
        quadratic_constraint);

    *ax = static_cast<float>(best_point.x() * translation_direction.x() / kMillimeterToMeter);
    *ay = static_cast<float>(best_point.x() * translation_direction.y() / kMillimeterToMeter);
    *alpha_z = static_cast<float>(best_point.y() * angular_direction);
}

constexpr double rmcs_core::controller::chassis::QcpSolver::nan_;
constexpr double rmcs_core::controller::chassis::QcpSolver::inf_;
constexpr double rmcs_core::controller::chassis::QcpSolver::epsilon_;
