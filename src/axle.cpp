#include "axle.h"
#include <algorithm>
#include <cmath>

namespace godot {

void Axle::_bind_methods() {
    
    ClassDB::bind_method(D_METHOD("set_steerable", "is_steerable"), &Axle::set_steerable);
    ClassDB::bind_method(D_METHOD("get_steerable"), &Axle::get_steerable);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_steerable"), "set_steerable", "get_steerable");

    ClassDB::bind_method(D_METHOD("set_drive_ratio", "drive_ratio"), &Axle::set_drive_ratio);
    ClassDB::bind_method(D_METHOD("get_drive_ratio"), &Axle::get_drive_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drive_ratio", PROPERTY_HINT_RANGE, "0.0,1.0,0.05"),
                          "set_drive_ratio", "get_drive_ratio");

    ClassDB::bind_method(D_METHOD("set_tire_data", "data"), &Axle::set_tire_data);
    ClassDB::bind_method(D_METHOD("get_tire_data"), &Axle::get_tire_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "tire_data", PROPERTY_HINT_RESOURCE_TYPE, "TireData"),
                          "set_tire_data", "get_tire_data");

    ClassDB::bind_method(D_METHOD("set_steering_rack_data", "data"), &Axle::set_steering_rack_data);
    ClassDB::bind_method(D_METHOD("get_steering_rack_data"), &Axle::get_steering_rack_data);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "steering_rack_data", PROPERTY_HINT_RESOURCE_TYPE, "SteeringRackData"), "set_steering_rack_data", "get_steering_rack_data");

    ClassDB::bind_method(D_METHOD("set_downforce_ratio", "value"), &Axle::set_downforce_ratio);
    ClassDB::bind_method(D_METHOD("get_downforce_ratio"), &Axle::get_downforce_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "downforce_ratio", PROPERTY_HINT_RANGE, "0.0,1.0,0.05"),
                          "set_downforce_ratio", "get_downforce_ratio");

    ClassDB::bind_method(D_METHOD("set_differential_type", "type"), &Axle::set_differential_type);
    ClassDB::bind_method(D_METHOD("get_differential_type"), &Axle::get_differential_type);
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "differential_type", PROPERTY_HINT_ENUM, "Open,LSD,Torsen"),
                          "set_differential_type", "get_differential_type");

    ClassDB::bind_method(D_METHOD("get_steer_angle"), &Axle::get_steer_angle);

}

Axle::Axle() {
    wheels = std::vector<Wheel*>();
}

void Axle::_ready() {

    if (Engine::get_singleton()->is_editor_hint())
        return;
    find_children_wheels();
    if (steering_rack_data != nullptr) {
        steering_rack.load(steering_rack_data);
    }
    // Apply tire data to wheels
    if (tire_data != nullptr) {
        for (auto* wh : wheels)
            wh->set_tire(tire_data);
    }
    _instantiate_differential();
}

const std::vector<Wheel*>& Axle::get_wheels() const {
    return wheels;
}

void Axle::find_children_wheels() {
    TypedArray<Node> arr = get_children();
    for (int i = 0; i < arr.size(); i++) {
        Wheel *iter = Object::cast_to<Wheel>(arr[i]);
        if (iter != nullptr) 
            wheels.push_back(iter);
    }
}

void Axle::compute_suspension_parameters(real_t mass, const Ref<SuspensionData>& s) {
    if (s.is_null()) {
        UtilityFunctions::printerr("There is no suspension data | axle.cpp script");
        return;
    }
    constexpr real_t gravity = 9.81;
    if (wheels.empty()) {
        UtilityFunctions::printerr("cannot compute axle values because there aren't any set raycasts for wheels | axle.cpp script");
        return;
    }
    real_t mass_per_wheel = mass / wheels.size();
    real_t average_wheel_load = mass_per_wheel * gravity;

    real_t target_compression = s->suspension_length * s->rest_compression;
    real_t stiffness = average_wheel_load / target_compression;
    real_t critical_damping = 2.0 * std::sqrt(stiffness * mass_per_wheel);
    real_t damping = s->damping_ratio * critical_damping;
    for (auto& i : wheels)
        i->set_suspension(s->suspension_length, stiffness, damping);
    arb_stiffness = s->antiroll_bar_stiffness;
}


void Axle::update_physics(PhysicsDirectBodyState3D *vehicle_state, const Vector3 &com_global, 
                          const Vector3 &linear_velocity, const Vector3 &angular_velocity) {
    for (auto& wheel : wheels)
        wheel->update_suspension(vehicle_state, com_global, linear_velocity, angular_velocity);
}

void Axle::solve_tire(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                      const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t dt,
                      real_t brake_input, bool abs_enabled) {

    // Update differential torque distribution based on latest wheel speeds
    if (differential) {
        differential->update(wheels);
    }

    for (auto &wheel : wheels)
        wheel->solve_tire(vehicle_state, com_global, linear_velocity, angular_velocity, dt, 
                          brake_input, abs_enabled);
}

void Axle::add_torque(real_t torque) {
    if (differential) {
        const auto &dist = differential->update(wheels);
        for (size_t i = 0; i < wheels.size() && i < 4; ++i) {
            wheels[i]->body.add_torque(torque * dist[i]);
        }
    } else {
        real_t per_wheel = wheels.empty() ? 0.0 : torque / static_cast<real_t>(wheels.size());
        for (auto &wheel : wheels)
            wheel->body.add_torque(per_wheel);
    }
}

real_t Axle::get_average_wheel_omega() const {
    if (wheels.empty()) return 0.0;
    real_t sum = 0.0;
    for (const auto& w : wheels)    sum += w->body.get_angular_velocity();
    return sum / wheels.size();
}

real_t Axle::get_total_sat() const {
    real_t value = 0.0;
    for (const auto& i : wheels)    value += i->self_aligning_torque;
    return value;
}


void Axle::integrate(real_t dt) {
    for (auto& wheel : wheels)  wheel->body.integrate(dt);
}
void Axle::solve_steering(real_t steer_input, real_t dt, real_t speed_kph) {
    real_t sat = get_total_sat();
    real_t speed_factor = std::min(speed_kph / real_t{8.0}, real_t{1.0});
    sat *= speed_factor;

    real_t driver_blend = real_t{1.0} - std::abs(steer_input);
    sat *= driver_blend;

    // low pass
    real_t alpha = std::min(dt * real_t{60.0}, real_t{1.0});
    filtered_sat += (sat - filtered_sat) * alpha;

    steering_rack.solve(steer_input, filtered_sat, dt, speed_kph);
}

real_t Axle::get_steer_angle() const {
    return steering_rack.get_angle();
}

void Axle::set_differential_type(const String &p_type) {
    differential_type = p_type;
    if (!Engine::get_singleton()->is_editor_hint()) {
        _instantiate_differential();
    }
}

String Axle::get_differential_type() const {
    return differential_type;
}

void Axle::_instantiate_differential() {
    if (differential_type == "LSD") {
        differential = std::make_unique<LSD_Differential>();
    } else if (differential_type == "Torsen") {
        differential = std::make_unique<Torsen_Differential>();
    } else {
        differential = std::make_unique<Open_Differential>();
    }
}

void Axle::set_downforce_ratio(real_t value) {
    downforce_ratio = std::clamp(value, real_t{0.0}, real_t{1.0});
}

real_t Axle::get_antiroll_bar_force() const {
    if (wheels.size() < 2 || arb_stiffness <= 0.0)
        return 0.0;

    return (wheels[1]->compression - wheels[0]->compression) * arb_stiffness;
}

void Axle::set_wheels_rotation() {
    if (wheels.size() != 2 || wheelbase < 0.001) {
        for (auto& wheel : wheels) {
            Vector3 r = wheel->get_rotation();
            r.y = steering_rack.get_angle();
            wheel->set_rotation(r);
        }
        return;
    }

    real_t steer_angle = steering_rack.get_angle();
    real_t abs_angle = std::abs(steer_angle);

    if (abs_angle < 0.001) {
        for (auto& wheel : wheels) {
            Vector3 r = wheel->get_rotation();
            r.y = 0.0;
            wheel->set_rotation(r);
        }
        return;
    }

    Wheel* left = wheels[0];
    Wheel* right = wheels[1];
    if (left->get_global_position().x > right->get_global_position().x)
        std::swap(left, right);

    // Ackermann formulas
    real_t R = wheelbase / std::tan(abs_angle);
    real_t half_track = trackwidth * real_t{0.5};
    real_t inner_radius = std::max(R - half_track, real_t{0.01});
    real_t outer_radius = R + half_track;

    real_t inner_angle = std::atan2(wheelbase, inner_radius);
    real_t outer_angle = std::atan2(wheelbase, outer_radius);

    if (steer_angle > 0) {
        right->set_rotation(Vector3(0, inner_angle, 0));
        left->set_rotation(Vector3(0, outer_angle, 0));
    } else {
        left->set_rotation(Vector3(0, -inner_angle, 0));
        right->set_rotation(Vector3(0, -outer_angle, 0));
    }
}

}
