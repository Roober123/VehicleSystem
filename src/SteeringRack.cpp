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
    angle = angular_velocity = target = raw_feedback = applied_feedback = 0.0;
    max_feedback_deflection = Math::deg_to_rad(s->get_max_feedback_deflection_deg());
    minimum_driver_authority = s->get_minimum_driver_authority();
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
    target = target_angle;
    const real_t driver_torque = stiffness * error;
    raw_feedback = sat_torque * sat_gain;
    const real_t absolute_limit = stiffness * max_feedback_deflection;
    const real_t authority_limit = (real_t{1.0} - minimum_driver_authority) * std::abs(driver_torque);
    // Smooth minimum never exceeds either bound; apply symmetrically to SAT.
    const real_t limit = absolute_limit > real_t{0.0} && authority_limit > real_t{0.0}
        ? absolute_limit * (authority_limit / std::hypot(absolute_limit, authority_limit))
        : real_t{0.0};
    applied_feedback = limit > real_t{0.0} ? limit * std::tanh(raw_feedback / limit) : real_t{0.0};
    sat_torque = applied_feedback;
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
