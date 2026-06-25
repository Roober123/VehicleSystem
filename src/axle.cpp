#include "axle.h"

namespace godot {

void Axle::_bind_methods() {
    
    ClassDB::bind_method(D_METHOD("set_steerable", "is_steerable"), &Axle::set_steerable);
    ClassDB::bind_method(D_METHOD("get_steerable"), &Axle::get_steerable);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "is_steerable"), "set_steerable", "get_steerable");

    ClassDB::bind_method(D_METHOD("set_drive_ratio", "drive_ratio"), &Axle::set_drive_ratio);
    ClassDB::bind_method(D_METHOD("get_drive_ratio"), &Axle::get_drive_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drive_ratio", PROPERTY_HINT_RANGE, "0.0,1.0,0.05"),
                          "set_drive_ratio", "get_drive_ratio");

}

Axle::Axle() {
    wheels = std::vector<Wheel*>();
}

void Axle::_ready() {

    if (Engine::get_singleton()->is_editor_hint())
        return;
    find_children_wheels();
    
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

void Axle::set_steerable(bool value) {
    is_steerable = value;
}

bool Axle::get_steerable() const {
    return is_steerable;
}

void Axle::set_drive_ratio(real_t value) {
    drive_ratio = value;
}

real_t Axle::get_drive_ratio() const {
    return drive_ratio;
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
    for (auto& i : wheels) {
        i->set_suspension(s->suspension_length, stiffness, damping);
    }
}


void Axle::update_physics(PhysicsDirectBodyState3D *vehicle_state, const Vector3 &com_global, const Vector3 &linear_velocity, const Vector3 &angular_velocity) {
    for (auto& wheel : wheels)
        wheel->update_suspension(vehicle_state, com_global, linear_velocity, angular_velocity);
}
void Axle::solve_tire(PhysicsDirectBodyState3D* vehicle_state, const Vector3 &com_global, 
                      const Vector3 &linear_velocity, const Vector3 &angular_velocity, real_t dt,
                      real_t brake_input) {
    for (auto &wheel : wheels)
        wheel->solve_tire(vehicle_state, com_global, linear_velocity, angular_velocity, dt, brake_input);
}

void Axle::add_torque(real_t torque) {
    if (wheels.size() > 0)
        torque /= wheels.size();
    for (auto& wheel : wheels)
        wheel->body.add_torque(torque);
}

real_t Axle::get_average_wheel_omega() const {
    if (wheels.empty())
        return 0.0;
    real_t sum = 0.0;
    for (const auto& w : wheels)
        sum += w->body.get_angular_velocity();
    return sum / wheels.size();
}

real_t Axle::get_total_sat() const {
    real_t value = 0.0;
    for (const auto& i : wheels)
        value += i->self_aligning_torque;
    return value;
}


void Axle::integrate(real_t dt) {
    for (auto& wheel : wheels)
        wheel->body.integrate(dt);
}


} // namespace godot
