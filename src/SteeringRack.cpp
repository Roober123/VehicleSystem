#include "SteeringRack.h"
#include <algorithm>

namespace godot {

void SteeringRack::load(const Ref<SteeringRackData>& s) {
    configured = false;
    if (s.is_null())
        return;

    inertia = std::max(s->get_inertia(), real_t{1e-6});
    damping = s->get_damping();
    friction_coefficient = s->get_friction_coefficient();
    max_angle = Math::deg_to_rad(s->get_max_angle());
    proportional_gain = s->get_proportional_gain();
    derivative_gain = s->get_derivative_gain();
    sat_gain = s->get_sat_gain();
    configured = true;
}

void SteeringRack::solve(real_t steer_input, real_t sat_torque, real_t dt, real_t speed_kph) {
    if (!configured || dt <= real_t{0.0})
        return;

    real_t speed_factor = 1.0 / (1.0 + speed_kph * real_t{0.02});

    real_t effective_steer = steer_input * speed_factor;
    real_t target_angle = effective_steer * max_angle;
    real_t error = target_angle - angle;
    real_t driver_torque = proportional_gain * error - derivative_gain * angular_velocity;
    sat_torque *= sat_gain;
    // Cap SAT so it cannot exceed driver authority (prevents steering lockup)
    real_t max_sat = proportional_gain * max_angle * real_t{0.8};
    sat_torque = std::clamp(sat_torque, -max_sat, max_sat);

    bool sat_opposes = (sat_torque > real_t{0.0}) != (driver_torque > real_t{0.0});
    if (sat_opposes) {
        real_t sat_limit = std::abs(driver_torque) * real_t{0.3};
        sat_torque = std::clamp(sat_torque, -sat_limit, sat_limit);
    }
    real_t friction_torque = friction_coefficient * tanh(angular_velocity * 5.0);
    real_t total_torque = driver_torque + sat_torque - friction_torque;
    real_t acceleration = total_torque / inertia;
    angular_velocity = (angular_velocity + acceleration * dt) / (1.0 + damping * dt / inertia);
    angle += angular_velocity * dt;
    if (angle > max_angle) {
        angle = max_angle;
        if (angular_velocity > 0.0)
            angular_velocity = 0.0;
    }
    if (angle < -max_angle) {
        angle = -max_angle;
        if (angular_velocity < 0.0)
            angular_velocity = 0.0;
    }

}

real_t SteeringRack::get_angle() const {
    return angle;
}

}
