#include "VehicleAerodynamics.h"
#include "axle.h"
#include <algorithm>
#include <cmath>

namespace godot {

AerodynamicsState VehicleAerodynamics::get_state(const Basis &body_basis,
                                                 const Vector3 &linear_velocity,
                                                 const Vector3 &angular_velocity,
                                                 real_t vehicle_mass,
                                                 const std::vector<Axle*> &axles) const {
    AerodynamicsState state;
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

AerodynamicForces VehicleAerodynamics::compute(const AerodynamicsState &state) const {
    const real_t speed_sq = state.linear_velocity.length_squared();
    const real_t dynamic_pressure = real_t{0.5} * air_density * speed_sq;

    AerodynamicForces forces;
    forces.drag = _compute_drag(state.linear_velocity, dynamic_pressure);
    forces.yaw_control_torque = _compute_yaw_control(state);
    forces.downforce = dynamic_pressure * downforce_coefficient * frontal_area;
    return forces;
}

Vector3 VehicleAerodynamics::_compute_drag(const Vector3 &linear_velocity, real_t dynamic_pressure) const {
    const real_t speed = linear_velocity.length();
    if (speed <= real_t{0.1})
        return Vector3();

    const real_t magnitude = dynamic_pressure * drag_coefficient * frontal_area;
    return -linear_velocity / speed * magnitude;
}

Vector3 VehicleAerodynamics::_compute_yaw_control(const AerodynamicsState &state) const {
    if (state.wheelbase <= real_t{0.01} || state.grounded_wheels < 2)
        return Vector3();

    const Vector3 local_velocity = state.body_basis.xform_inv(state.linear_velocity);
    const real_t forward_speed = local_velocity.z;
    const real_t abs_forward_speed = std::abs(forward_speed);
    if (abs_forward_speed < yaw_control_min_speed)
        return Vector3();

    real_t desired_yaw_rate = forward_speed / state.wheelbase * std::tan(state.steer_angle);
    const real_t max_yaw_rate = yaw_control_max_lateral_acceleration /
                                std::max(abs_forward_speed, real_t{1.0});
    desired_yaw_rate = std::clamp(desired_yaw_rate, -max_yaw_rate, max_yaw_rate);

    const Vector3 local_angular_velocity = state.body_basis.xform_inv(state.angular_velocity);
    const real_t yaw_error = local_angular_velocity.y - desired_yaw_rate;
    const real_t yaw_inertia = state.vehicle_mass *
        (state.wheelbase * state.wheelbase + state.trackwidth * state.trackwidth) / real_t{12.0};
    real_t torque = -yaw_inertia * yaw_damping_coefficient * yaw_error;
    torque = std::clamp(torque, -yaw_control_max_torque, yaw_control_max_torque);

    return state.body_basis.get_column(1) * torque;
}

void VehicleAerodynamics::load_parameters(const Ref<VehicleAerodynamicsData>& a) {
    if (a.is_null()) {
        drag_coefficient = 0.0;
        downforce_coefficient = 0.0;
        yaw_damping_coefficient = 0.0;
        yaw_control_min_speed = 0.0;
        yaw_control_max_torque = 0.0;
        yaw_control_max_lateral_acceleration = 0.0;
        return;
    }
    drag_coefficient = a->get_drag_coefficient();
    downforce_coefficient = a->get_downforce_coefficient();
    yaw_damping_coefficient = a->get_yaw_damping_coefficient();
    yaw_control_min_speed = a->get_yaw_control_min_speed();
    yaw_control_max_torque = a->get_yaw_control_max_torque();
    yaw_control_max_lateral_acceleration = a->get_yaw_control_max_lateral_acceleration();
}

} // namespace godot
