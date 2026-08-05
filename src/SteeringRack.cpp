#include "SteeringRack.h"
#include <algorithm>
#include <cmath>

namespace godot {

void SteeringRack::load(const Ref<SteeringRackData>& s) {
    if (s.is_null())
        return;

    inertia = std::max(s->get_inertia(), real_t{1e-6});
    damping = s->get_damping();
    friction_coefficient = s->get_friction_coefficient();
    max_angle = Math::deg_to_rad(s->get_max_angle());
    proportional_gain = s->get_proportional_gain();
    derivative_gain = s->get_derivative_gain();
    sat_gain = s->get_sat_gain();
}

void SteeringRack::solve(real_t steer_input, real_t sat_torque, real_t dt, real_t speed_kph) {
    if (!std::isfinite(dt) || dt <= real_t{0.0})
        return;

    real_t speed_factor = 1.0 / (1.0 + speed_kph * real_t{0.02});

    real_t effective_steer = steer_input * speed_factor;
    real_t target_angle = effective_steer * max_angle;
    real_t error = target_angle - angle;
    real_t driver_torque = proportional_gain * error - derivative_gain * angular_velocity;
    sat_torque *= sat_gain;

    // SAT may assist the player/PD torque without an artificial cap.  With no
    // player/PD torque, SAT has no authority to initiate rack motion.  When
    // it opposes a non-zero command, cap only its magnitude so the combined
    // command cannot reverse the player's authority.
    if (driver_torque == real_t{0.0}) {
        sat_torque = real_t{0.0};
    } else if ((sat_torque > real_t{0.0}) != (driver_torque > real_t{0.0})) {
        sat_torque = std::copysign(
            std::min(std::abs(sat_torque), std::abs(driver_torque)), sat_torque);
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
