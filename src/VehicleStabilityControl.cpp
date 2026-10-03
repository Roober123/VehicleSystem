#include "VehicleStabilityControl.h"
#include "axle.h"

#include <algorithm>
#include <cmath>

namespace godot {

void VehicleStabilityControl::load_parameters(const Ref<ESCData> &data) {
    if (resource != data)
        reset();
    resource = data;
    enabled = data.is_valid() && data->get_enabled();
    if (data.is_null()) {
        reset();
        return;
    }
    if (!enabled) reset();
    smoothing_enabled = data->get_torque_smoothing_enabled();
    engagement_rate = data->get_torque_engagement_rate();
    release_rate = data->get_torque_release_rate();
    yaw_damping = data->get_yaw_damping();
    minimum_speed = data->get_minimum_speed();
    maximum_corrective_torque = data->get_maximum_corrective_torque();
    maximum_target_lateral_acceleration = data->get_maximum_target_lateral_acceleration();
}

StabilityState VehicleStabilityControl::get_state(const Basis &body_basis,
                                                 const Vector3 &linear_velocity,
                                                 const Vector3 &angular_velocity,
                                                 real_t vehicle_mass,
                                                 const std::vector<Axle*> &axles) const {
    StabilityState state;
    state.body_basis = body_basis.orthonormalized();
    state.linear_velocity = linear_velocity;
    state.angular_velocity = angular_velocity;
    state.vehicle_mass = vehicle_mass;

    real_t trackwidth_sum = real_t{0.0};
    real_t steer_angle_sum = real_t{0.0};
    int track_count = 0;
    int steerable_count = 0;

    for (const auto *axle : axles) {
        state.wheelbase = std::max(state.wheelbase, axle->get_wheelbase());
        if (axle->get_trackwidth() > real_t{0.0}) {
            trackwidth_sum += axle->get_trackwidth();
            ++track_count;
        }
        if (axle->get_steerable()) {
            steer_angle_sum += axle->get_steer_angle();
            ++steerable_count;
        }
        for (const auto *wheel : axle->get_wheels()) {
            if (wheel->is_on_ground())
                ++state.grounded_wheels;
        }
    }

    if (track_count > 0)
        state.trackwidth = trackwidth_sum / static_cast<real_t>(track_count);
    if (steerable_count > 0)
        state.steer_angle = steer_angle_sum / static_cast<real_t>(steerable_count);

    return state;
}

bool VehicleStabilityControl::valid_state(const StabilityState &state) const {
    return state.body_basis.get_column(0).is_finite() &&
        state.body_basis.get_column(1).is_finite() && state.body_basis.get_column(2).is_finite() &&
        std::isfinite(state.body_basis.determinant()) && std::abs(state.body_basis.determinant()) > real_t{1e-6} &&
        state.linear_velocity.is_finite() && state.angular_velocity.is_finite() &&
        std::isfinite(state.vehicle_mass) && state.vehicle_mass > real_t{0.0} &&
        std::isfinite(state.wheelbase) && state.wheelbase > real_t{0.01} &&
        std::isfinite(state.trackwidth) && state.trackwidth >= real_t{0.0} && std::isfinite(state.steer_angle);
}

Vector3 VehicleStabilityControl::compute_torque(const StabilityState &state) const {
    return raw_torque(state, nullptr);
}

Vector3 VehicleStabilityControl::raw_torque(const StabilityState &state, StabilityTelemetry *sample) const {
    if (!enabled || !valid_state(state) || state.grounded_wheels < 2)
        return Vector3();

    const Vector3 local_velocity = state.body_basis.xform_inv(state.linear_velocity);
    const real_t forward_speed = local_velocity.z;
    const real_t abs_forward_speed = std::abs(forward_speed);
    if (sample) {
        sample->supported = true;
        sample->speed_gate = abs_forward_speed < minimum_speed;
    }
    if (abs_forward_speed < minimum_speed)
        return Vector3();

    real_t desired_yaw_rate = forward_speed / state.wheelbase * std::tan(state.steer_angle);
    const real_t max_yaw_rate = maximum_target_lateral_acceleration /
                                std::max(abs_forward_speed, real_t{1.0});
    desired_yaw_rate = std::clamp(desired_yaw_rate, -max_yaw_rate, max_yaw_rate);

    const Vector3 local_angular_velocity = state.body_basis.xform_inv(state.angular_velocity);
    const real_t yaw_error = local_angular_velocity.y - desired_yaw_rate;
    const real_t yaw_inertia = state.vehicle_mass *
        (state.wheelbase * state.wheelbase + state.trackwidth * state.trackwidth) / real_t{12.0};
    real_t torque = -yaw_inertia * yaw_damping * yaw_error;
    if (!std::isfinite(torque)) {
        if (sample) sample->supported = false;
        return Vector3();
    }
    if (sample) {
        sample->yaw_target = desired_yaw_rate;
        sample->yaw_error = yaw_error;
        sample->requested_torque = torque;
        sample->cap_active = std::abs(torque) > maximum_corrective_torque;
    }
    torque = std::clamp(torque, -maximum_corrective_torque, maximum_corrective_torque);

    return state.body_basis.get_column(1) * torque;
}

void VehicleStabilityControl::reset() {
    applied_torque = 0.0;
    telemetry = StabilityTelemetry();
}

Vector3 VehicleStabilityControl::update(const StabilityState &state, real_t dt) {
    // Observe resource edits/removal once per chassis tick, without advancing
    // output history when the same resource is reloaded.
    const Ref<ESCData> current_resource = resource;
    load_parameters(current_resource);
    if (!enabled || !valid_state(state) || state.grounded_wheels < 2) {
        reset();
        return Vector3();
    }
    if (!std::isfinite(dt) || dt <= real_t{0.0})
        return Vector3(); // no new correction or history advancement
    telemetry = StabilityTelemetry();
    const Vector3 up = state.body_basis.get_column(1);
    const real_t target = raw_torque(state, &telemetry).dot(up) / up.length_squared();
    if (!telemetry.supported || !std::isfinite(target)) {
        reset();
        return Vector3();
    }
    // A newly reduced cap is a hard bound, including during sign reversal.
    applied_torque = std::clamp(applied_torque, -maximum_corrective_torque, maximum_corrective_torque);
    if (!smoothing_enabled) {
        applied_torque = target;
    } else {
        real_t remaining = dt;
        if (applied_torque * target < real_t{0.0}) {
            const real_t release_time = std::abs(applied_torque) / release_rate;
            if (remaining < release_time) {
                applied_torque -= std::copysign(release_rate * remaining, applied_torque);
                remaining = 0.0;
            } else {
                applied_torque = 0.0;
                remaining -= release_time;
            }
        }
        if (remaining > real_t{0.0}) {
            const real_t rate = std::abs(target) > std::abs(applied_torque) ? engagement_rate : release_rate;
            const real_t change = rate * remaining;
            applied_torque += std::clamp(target - applied_torque, -change, change);
        }
    }
    applied_torque = std::clamp(applied_torque, -maximum_corrective_torque, maximum_corrective_torque);
    telemetry.applied_torque = applied_torque;
    telemetry.slew_active = applied_torque != target;
    const Vector3 output = up * applied_torque;
    if (!output.is_finite()) {
        reset();
        return Vector3();
    }
    return output;
}

} // namespace godot
