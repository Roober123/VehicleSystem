#include "SteeringRack.h"
#include <algorithm>
#include <cmath>

namespace godot {

void SteeringRack::load(const Ref<SteeringRackData>& s) {
    if (s.is_null())
        return;

    // For a critically damped step, 1-(1+w*t)*exp(-w*t) reaches 0.9
    // at w*t=3.88972017. Fixed inertia removes a redundant tuning dimension.
    constexpr real_t rack_inertia = real_t{0.3};
    response_frequency = real_t{3.88972017} / (s->get_response_time_ms() * real_t{0.001});
    stiffness = rack_inertia * response_frequency * response_frequency;
    friction_torque = s->get_friction_torque();
    max_angle = Math::deg_to_rad(s->get_max_angle());
    steering_half_speed_kph = s->get_steering_half_speed_kph();
    // Preserve the feedback deflection relative to driver stiffness when
    // response time changes. 400 Nm/rad is the reference steering stiffness.
    sat_gain = s->get_road_feedback_strength() * stiffness / real_t{400.0};
}

void SteeringRack::solve(real_t steer_input, real_t sat_torque, real_t dt, real_t speed_kph) {
    if (!std::isfinite(dt) || dt <= real_t{0.0} || stiffness <= real_t{0.0} ||
            !std::isfinite(steer_input) || !std::isfinite(sat_torque) || !std::isfinite(speed_kph))
        return;

    const real_t speed_factor = steering_half_speed_kph > real_t{0.0}
        ? real_t{1.0} / (real_t{1.0} + std::abs(speed_kph) / steering_half_speed_kph)
        : real_t{1.0};

    real_t effective_steer = std::clamp(steer_input, real_t{-1.0}, real_t{1.0}) * speed_factor;
    real_t target_angle = effective_steer * max_angle;
    real_t error = target_angle - angle;
    // Road feedback is bounded by the driver's restoring torque. Intrinsic
    // response damping remains active even when opposing feedback reaches
    // that bound; otherwise feedback could cancel damping and cause drift.
    real_t driver_torque = stiffness * error;
    sat_torque *= sat_gain;

    // SAT may assist the restoring torque without an artificial cap. With no
    // restoring torque, SAT has no authority to initiate rack motion. When
    // it opposes a non-zero command, cap only its magnitude so the combined
    // command cannot reverse the player's authority.
    if (driver_torque == real_t{0.0}) {
        sat_torque = real_t{0.0};
    } else if ((sat_torque > real_t{0.0}) != (driver_torque > real_t{0.0})) {
        sat_torque = std::copysign(
            std::min(std::abs(sat_torque), std::abs(driver_torque)), sat_torque);
    }
    const real_t resisting_torque = friction_torque * std::tanh(angular_velocity * real_t{5.0});
    integrate_response(target_angle, sat_torque - resisting_torque, dt);
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

void SteeringRack::integrate_response(real_t target_angle, real_t external_torque, real_t dt) {
    // Exact critically damped update for this step's target and road torque.
    // No-feedback response times therefore do not depend on substep count.
    const real_t equilibrium = target_angle + external_torque / stiffness;
    const real_t offset = angle - equilibrium;
    const real_t transient = angular_velocity + response_frequency * offset;
    const real_t decay = std::exp(-response_frequency * dt);
    angle = equilibrium + (offset + transient * dt) * decay;
    angular_velocity = (angular_velocity - response_frequency * transient * dt) * decay;
}

real_t SteeringRack::get_angle() const {
    return angle;
}

}
