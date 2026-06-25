#include "VehicleTelemetry.h"
#include "godot_cpp/variant/utility_functions.hpp"

namespace godot {

void VehicleTelemetry::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_target", "vehicle"), &VehicleTelemetry::set_target);
    ClassDB::bind_method(D_METHOD("get_target"), &VehicleTelemetry::get_target);

    ClassDB::bind_method(D_METHOD("get_engine_rpm"), &VehicleTelemetry::get_engine_rpm);
    ClassDB::bind_method(D_METHOD("get_engine_torque"), &VehicleTelemetry::get_engine_torque);
    ClassDB::bind_method(D_METHOD("get_engine_throttle"), &VehicleTelemetry::get_engine_throttle);
    ClassDB::bind_method(D_METHOD("get_driveshaft_rpm"), &VehicleTelemetry::get_driveshaft_rpm);
    ClassDB::bind_method(D_METHOD("get_driveshaft_twist"), &VehicleTelemetry::get_driveshaft_twist);
    ClassDB::bind_method(D_METHOD("get_clutch_engagement"), &VehicleTelemetry::get_clutch_engagement);
    ClassDB::bind_method(D_METHOD("get_current_gear"), &VehicleTelemetry::get_current_gear);
    ClassDB::bind_method(D_METHOD("get_gear_ratio"), &VehicleTelemetry::get_gear_ratio);
    ClassDB::bind_method(D_METHOD("get_vehicle_speed_kph"), &VehicleTelemetry::get_vehicle_speed_kph);
    ClassDB::bind_method(D_METHOD("get_wheel_angular_velocities"), &VehicleTelemetry::get_wheel_angular_velocities);
    ClassDB::bind_method(D_METHOD("get_tire_forces"), &VehicleTelemetry::get_tire_forces);
}

void VehicleTelemetry::_ready() {
    if (Engine::get_singleton()->is_editor_hint())
        return;

    // Auto-detect target: look for a Vehicle among siblings or parent
    if (target == nullptr) {
        // Try parent first
        target = Object::cast_to<Vehicle>(get_parent());
    }
    if (target == nullptr) {
        // Try siblings
        Node* parent = get_parent();
        if (parent != nullptr) {
            TypedArray<Node> children = parent->get_children();
            for (int i = 0; i < children.size(); i++) {
                Vehicle* v = Object::cast_to<Vehicle>(children[i]);
                if (v != nullptr) {
                    target = v;
                    break;
                }
            }
        }
    }
}

void VehicleTelemetry::_process(double delta) {
    if (target == nullptr)
        return;

    constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);

    // Engine
    VehicleEngine& eng = target->get_engine();
    engine_rpm = eng.get_angular_velocity() * ang_to_rpm;
    engine_torque = eng.get_torque();
    engine_throttle = target->get_throttle_input();

    // Driveshaft
    const RotationalBody& ds = target->get_driveshaft();
    driveshaft_rpm = ds.get_angular_velocity() * ang_to_rpm;

    // Clutch + gearbox
    const ClutchGearConstraint& cg = target->get_clutch_gearbox();
    clutch_engagement = cg.clutch_engagement;
    current_gear = cg.current_gear;
    gear_ratio = cg.get_effective_ratio();

    // Vehicle speed
    vehicle_speed_kph = target->get_speed_kph();

    // Wheel angular velocities (RPM)
    wheel_angular_velocities.clear();
    tire_forces.clear();
    for (const Axle* axle : target->axles) {
        for (const Wheel* w : axle->get_wheels()) {
            wheel_angular_velocities.push_back(w->get_angular_velocity() * ang_to_rpm);
            tire_forces.push_back(w->get_tire_force());
        }
    }
}

void VehicleTelemetry::set_target(Vehicle* p_vehicle) {
    target = p_vehicle;
}

Vehicle* VehicleTelemetry::get_target() const {
    return target;
}

}
