#include "wheel.h"
#include "godot_cpp/variant/utility_functions.hpp"
#include <algorithm>
#include <cmath>

namespace godot {

void Wheel::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_angular_velocity"), &Wheel::get_angular_velocity);
    ClassDB::bind_method(D_METHOD("get_tire_force"), &Wheel::get_tire_force);
    ClassDB::bind_method(D_METHOD("get_collision_point"), &Wheel::get_collision_point);
    ClassDB::bind_method(D_METHOD("get_collision_normal"), &Wheel::get_collision_normal);
    ClassDB::bind_method(D_METHOD("is_on_ground"), &Wheel::is_on_ground);
    ClassDB::bind_method(D_METHOD("get_slip_ratio"), &Wheel::get_slip_ratio);
}

void Wheel::set_suspension(real_t suspension_length, real_t stiffness, real_t damping, real_t reference_load) {
    this->suspension_length = suspension_length;
    this->stiffness = stiffness;
    this->damping = damping;
    this->reference_load = std::max(reference_load, real_t{1.0});
    add_physics();
    if (skid != nullptr)
        skid->set_ribbon_width(tire_width);

} 
void Wheel::set_tire(const Ref<TireData>& t) {
    if (t == nullptr)
        return;
    this->friction_forward = t->friction_forward;
    this->friction_lateral = t->friction_lateral;
    this->forward_friction_curve = t->get_forward_friction_curve();
    this->lateral_friction_curve = t->get_lateral_friction_curve();
    this->radius = t->radius;
    this->tire_width = t->tire_width;
    this->brake_power = t->brake_power;
    this->peak_slip_angle = t->peak_slip_angle;
    this->relaxation_low = t->relaxation_low;
    this->relaxation_high = t->relaxation_high;
    this->load_sensitivity = t->get_load_sensitivity();

    // Pass tire width to the skid system
    if (skid)
        skid->set_ribbon_width(t->tire_width);

    constexpr real_t patch_length = real_t{0.4};
    body.set_inertia(patch_length * radius * radius * real_t{25.0});
    body.set_drag(t->get_drag());
}

void Wheel::add_physics() {
    if (ray == nullptr) {
        ray = memnew(RayCast3D);
        add_child(ray);
        ray->set_enabled(false);
    }
    ray->set_target_position(Vector3(0, -suspension_length, 0));

    // Create TireSkid as a child node
    if (skid == nullptr) {
        skid = memnew(TireSkid);
        skid->set_name("TireSkid");
        add_child(skid);
    }
}

void Wheel::update_suspension(const Vector3 &com_global,
                              const Vector3 &linear_velocity,
                              const Vector3 &angular_velocity) {
    ray->force_raycast_update();
    if (!ray->is_colliding()) {
        on_ground = false;
        set_normal_force(0.0);
        compression = 0.0;
        collision_point = get_global_position();
        collision_normal = Vector3();
        reaction_torque = 0.0f;
        self_aligning_torque = 0.0;
        slip_ratio = 0.0;
        slip_angle = 0.0;
        is_sliding = false;
        abs_active = false;
        prev_longitudinal_force = 0.0;
        prev_lateral_force = 0.0;
        if (skid != nullptr)
            skid->stop_skid();
        return;
    }
    on_ground = true;
    collision_point = ray->get_collision_point();
    collision_normal = ray->get_collision_normal();
    compression = std::clamp(
        suspension_length - ray->get_global_position().distance_to(collision_point),
        real_t{0.0}, suspension_length);
    
    Vector3 velocity_at_point = linear_velocity + angular_velocity.cross(
                                (collision_point - com_global));
    real_t velocity_along_normal = velocity_at_point.dot(collision_normal);

    const real_t spring_force = compression * stiffness;
    const real_t damper_force = -damping * velocity_along_normal;
    set_normal_force(spring_force + damper_force);

    forward_vector = get_global_transform().basis.get_column(2); // Z forward
    right_vector = get_global_transform().basis.get_column(0); // X right
}

void Wheel::set_normal_force(real_t force) {
    suspension_rebound_force = std::max(force, real_t{0.0});
}


void Wheel::solve_tire(const Vector3 &com_global, const Vector3 &linear_velocity,
                       const Vector3 &angular_velocity, real_t dt, real_t brake_input,
                       bool abs_enabled) {
    is_sliding = false;
    reaction_torque = 0.0f;
    self_aligning_torque = 0.0;
    if (!on_ground)
        return;

    Vector3 fwd_tangent, right_tangent;
    _compute_tangents(fwd_tangent, right_tangent);

    Vector3 vel_point = linear_velocity + angular_velocity.cross(collision_point - com_global);
    real_t fwd_speed = vel_point.dot(fwd_tangent);
    real_t lat_speed = vel_point.dot(right_tangent);

    real_t slip_vel, slip_angle_rad;
    _compute_slip(fwd_speed, lat_speed, slip_vel, slip_angle_rad);

    real_t normal = _compute_normal_force();
    if (normal <= real_t{1e-6}) {
        prev_longitudinal_force = 0.0;
        prev_lateral_force = 0.0;
        _apply_brakes(brake_input, abs_enabled, fwd_speed, dt, normal, fwd_tangent);
        return;
    }

    real_t raw_fwd_force, raw_lat_force, fwd_mu, lat_mu;
    _compute_raw_forces(normal, slip_vel, slip_angle_rad, fwd_speed, lat_speed,
                        raw_fwd_force, raw_lat_force, fwd_mu, lat_mu);

    real_t longitudinal_force, lateral_force, sum;
    sum = _combine_forces(raw_fwd_force, raw_lat_force, normal, fwd_mu, lat_mu,
                          longitudinal_force, lateral_force);

    _apply_relaxation(longitudinal_force, lateral_force, dt, linear_velocity);

    _apply_tire_forces(fwd_tangent, right_tangent, longitudinal_force, lateral_force);

    _compute_sat(lateral_force);
    body.add_torque(reaction_torque);

    _detect_tire_instability(lateral_force, dt);

    _update_skidmarks(vel_point, sum, dt);

    _apply_brakes(brake_input, abs_enabled, fwd_speed, dt, normal, fwd_tangent);
}

void Wheel::_detect_tire_instability(real_t total_lateral_force, real_t dt) {
    instability_cooldown -= dt;

    constexpr real_t force_threshold = 500.0;
    constexpr real_t cooldown_time = 0.5;

    if (std::abs(total_lateral_force) > force_threshold &&
        std::abs(prev_lateral_force) > force_threshold) {
        if ((total_lateral_force > 0.0) != (prev_lateral_force > 0.0)) {
            oscillation_count++;
            if (instability_cooldown <= 0.0) {
                String wheel_name = get_name();
                if (wheel_name.is_empty()) wheel_name = "(unnamed)";
                UtilityFunctions::print(
                    "[TireInstability] Wheel '", wheel_name, "'"
                    " | lateral oscillation #", oscillation_count,
                    " | prev=", prev_lateral_force,
                    " | curr=", total_lateral_force,
                    " | sustained_mass=", suspension_rebound_force / real_t{9.81},
                    " | compression=", compression);
                instability_cooldown = cooldown_time;
            }
        }
    }
    prev_lateral_force = total_lateral_force;
}

real_t Wheel::_apply_abs(real_t brake_input, real_t fwd_speed, real_t dt) {
    abs_active = false;
    if (brake_input <= 0.0f || std::abs(fwd_speed) <= real_t{0.5}) {
        abs_accumulator = real_t{1.0};
        return brake_input;
    }

    constexpr real_t abs_target_slip = real_t{0.15};
    constexpr real_t abs_release_fraction = real_t{0.25};
    constexpr real_t abs_cycle_rate = real_t{8.0}; // Hz, pressure rebuild rate

    real_t slip_error = std::abs(slip_ratio) - abs_target_slip;

    if (slip_error > 0.0f) {
        // Wheel is locking — release pressure proportionally
        real_t release = std::min(slip_error * real_t{4.0}, real_t{1.0});
        real_t effective_brake = brake_input * (real_t{1.0} - release * (real_t{1.0} - abs_release_fraction));
        abs_accumulator = std::max(abs_accumulator - dt * abs_cycle_rate, real_t{0.0});
        abs_active = true;
        return effective_brake;
    }

    abs_accumulator = std::min(abs_accumulator + dt * abs_cycle_rate, real_t{1.0});
    return brake_input * abs_release_fraction +
           brake_input * (real_t{1.0} - abs_release_fraction) * abs_accumulator;
}

void Wheel::_compute_sat(real_t lateral_force) {
    constexpr real_t base_trail = real_t{0.04}; // 40 mm
    const real_t normal = _compute_normal_force();
    real_t max_lateral = normal * friction_lateral * _get_load_sensitivity_scale(normal);
    real_t load_ratio = std::abs(lateral_force) / std::max(max_lateral, real_t{1e-6});
    load_ratio = std::min(load_ratio, real_t{1.0});
    real_t trail = base_trail * (real_t{1.0} - load_ratio) * (real_t{1.0} - load_ratio);
    self_aligning_torque = -trail * lateral_force;
}

void Wheel::_compute_tangents(Vector3& fwd_tangent, Vector3& right_tangent) const {
    fwd_tangent = (forward_vector - collision_normal * forward_vector.dot(collision_normal)).normalized();
    right_tangent = (right_vector - collision_normal * right_vector.dot(collision_normal)).normalized();
}

void Wheel::_compute_slip(real_t fwd_speed, real_t lat_speed, real_t& slip_vel, real_t& slip_angle_rad) {
    real_t ang_vel = get_angular_velocity();
    slip_vel = ang_vel * radius - fwd_speed;

    slip_angle_rad = atan2(lat_speed, std::abs(fwd_speed) + real_t{2.5});

    slip_ratio = slip_vel / std::max(std::abs(fwd_speed), real_t{1.0});
    slip_angle = slip_angle_rad * real_t{180.0} / Math_PI;

    constexpr real_t sliding_slip_ratio = real_t{0.2};
    constexpr real_t sliding_slip_angle = real_t{6.0};
    is_sliding = std::abs(slip_ratio) > sliding_slip_ratio ||
                 std::abs(slip_angle) > sliding_slip_angle;
}

real_t Wheel::_compute_normal_force() const {
    return suspension_rebound_force;
}

void Wheel::_compute_raw_forces(real_t normal, real_t slip_vel, real_t slip_angle_rad,
                                 real_t fwd_speed, real_t lat_speed,
                                 real_t& raw_fwd_force, real_t& raw_lat_force,
                                 real_t& fwd_mu, real_t& lat_mu) const {
    constexpr real_t peak_slip_vel = real_t{3.0};
    real_t peak_slip_rad = peak_slip_angle * Math_PI / real_t{180.0};

    fwd_mu = friction_forward;
    lat_mu = friction_lateral;

    const real_t load_scale = _get_load_sensitivity_scale(normal);
    fwd_mu *= load_scale;
    lat_mu *= load_scale;

    if (std::abs(fwd_speed) + std::abs(lat_speed) < real_t{1.0}) {
        fwd_mu *= real_t{1.3};
        lat_mu *= real_t{1.3};
    }


    if (forward_friction_curve.is_valid()) {
        real_t t = std::clamp(std::abs(slip_vel) / peak_slip_vel, real_t{0.0}, real_t{2.0});
        real_t mult = forward_friction_curve->sample(t);
        raw_fwd_force = normal * fwd_mu * mult * (slip_vel >= real_t{0.0} ? real_t{1.0} : real_t{-1.0});
    } else {
        raw_fwd_force = normal * fwd_mu * tanh(slip_vel / peak_slip_vel);
    }

    if (lateral_friction_curve.is_valid()) {
        real_t t = std::clamp(std::abs(slip_angle_rad) / peak_slip_rad, real_t{0.0}, real_t{2.0});
        real_t mult = lateral_friction_curve->sample(t);
        raw_lat_force = normal * lat_mu * mult * (slip_angle_rad >= real_t{0.0} ? real_t{1.0} : real_t{-1.0});
    } else {
        raw_lat_force = normal * lat_mu * tanh(slip_angle_rad / peak_slip_rad);
    }
}

real_t Wheel::_combine_forces(real_t raw_fwd, real_t raw_lat, real_t normal,
                               real_t fwd_mu, real_t lat_mu,
                               real_t& out_fwd, real_t& out_lat) const {
    constexpr real_t min_mu = real_t{1e-6};
    real_t nx = lat_mu > min_mu ? raw_lat / (lat_mu * normal) : real_t{0.0};
    real_t ny = fwd_mu > min_mu ? raw_fwd / (fwd_mu * normal) : real_t{0.0};
    real_t sum = std::sqrt(nx * nx + ny * ny);

    out_fwd = raw_fwd;
    out_lat = raw_lat;
    if (sum > real_t{1.0}) {
        real_t r = real_t{1.0} / sum;
        out_fwd *= r;
        out_lat *= r;
    }
    return sum;
}

real_t Wheel::_get_load_sensitivity_scale(real_t normal_load) const {
    const real_t load_ratio = std::max(normal_load / reference_load, real_t{1e-3});
    const real_t scale = std::pow(load_ratio, -load_sensitivity);
    return std::clamp(scale, real_t{0.75}, real_t{1.25});
}

void Wheel::_apply_relaxation(real_t& longitudinal_force, real_t& lateral_force,
                               real_t dt, const Vector3& linear_velocity) {
    real_t clamped_speed = std::clamp(std::abs(linear_velocity.length()), real_t{0.0}, real_t{30.0});
    real_t relaxation_time = relaxation_low + (relaxation_high - relaxation_low) * clamped_speed / real_t{30.0};
    real_t alpha = std::min(dt / std::max(relaxation_time, real_t{1e-6}), real_t{1.0});

    longitudinal_force = prev_longitudinal_force + (longitudinal_force - prev_longitudinal_force) * alpha;
    lateral_force = prev_lateral_force + (lateral_force - prev_lateral_force) * alpha;

    prev_longitudinal_force = longitudinal_force;
    prev_lateral_force = lateral_force;
}

void Wheel::_apply_tire_forces(const Vector3& fwd_tangent, const Vector3& right_tangent,
                                real_t longitudinal_force, real_t lateral_force) {
    Vector3 point_force = fwd_tangent * longitudinal_force - right_tangent * lateral_force;
    tire_force += point_force;
    reaction_torque -= longitudinal_force * radius;
}

void Wheel::_update_skidmarks(const Vector3& vel_point, real_t friction_sum, real_t dt) {
    if (!skid) return;

    if (friction_sum > 0.99) {
        skid_stop_cooldown = real_t{0.12};
        Vector3 vel_on_ground = vel_point - collision_normal * vel_point.dot(collision_normal);
        skid->update_skid(collision_point, collision_normal, vel_on_ground);
    } else {
        skid_stop_cooldown -= dt;
        if (skid_stop_cooldown <= real_t{0.0}) {
            skid_stop_cooldown = real_t{0.0};
            skid->stop_skid();
        }
    }
}

void Wheel::_apply_brakes(real_t brake_input, bool abs_enabled, real_t fwd_speed,
                           real_t dt, real_t normal, const Vector3& fwd_tangent) {
    real_t effective_brake = brake_input;
    if (abs_enabled) effective_brake = _apply_abs(brake_input, fwd_speed, dt);

    if (effective_brake > 0.0f) {
        const real_t omega = get_angular_velocity();
        const real_t brake_torque = effective_brake * brake_power;
        const real_t max_stopping = std::abs(omega) * body.get_inertia() / std::max(dt, 1e-6f);

        if (brake_torque >= max_stopping) {
            body.set_angular_velocity(0.0);
            body.clear_torque();
            // hold on hill
            if (std::abs(fwd_speed) > real_t{0.005}) {
                real_t max_hold = normal * friction_forward * effective_brake;
                real_t hold_force = -tanh(fwd_speed * real_t{20.0}) * max_hold;
                tire_force += fwd_tangent * hold_force;
            }
        } else {
            body.add_torque((omega > 0.0f) ? -brake_torque : brake_torque);
        }
    }
}

}
