#include "VehicleTelemetry.h"
#include "godot_cpp/classes/engine.hpp"

#include <algorithm>

namespace godot {

void VehicleTelemetry::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_target", "vehicle"), &VehicleTelemetry::set_target);
    ClassDB::bind_method(D_METHOD("get_target"), &VehicleTelemetry::get_target);

    ClassDB::bind_method(D_METHOD("get_engine_rpm"), &VehicleTelemetry::get_engine_rpm);
    ClassDB::bind_method(D_METHOD("get_engine_torque"), &VehicleTelemetry::get_engine_torque);
    ClassDB::bind_method(D_METHOD("get_engine_throttle"), &VehicleTelemetry::get_engine_throttle);
    ClassDB::bind_method(D_METHOD("get_driveshaft_rpm"), &VehicleTelemetry::get_driveshaft_rpm);
    ClassDB::bind_method(D_METHOD("get_clutch_engagement"), &VehicleTelemetry::get_clutch_engagement);
    ClassDB::bind_method(D_METHOD("get_current_gear"), &VehicleTelemetry::get_current_gear);
    ClassDB::bind_method(D_METHOD("get_gear_ratio"), &VehicleTelemetry::get_gear_ratio);
    ClassDB::bind_method(D_METHOD("get_vehicle_speed_kph"), &VehicleTelemetry::get_vehicle_speed_kph);
    ClassDB::bind_method(D_METHOD("get_turbo_boost"), &VehicleTelemetry::get_turbo_boost);
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

    cache_wheel_views();
}

void VehicleTelemetry::_process(double delta) {
    (void)delta;
    if (target == nullptr)
        return;

    constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);

    snapshot = target->get_telemetry_snapshot();

    const std::vector<Axle *> &axle_views = target->get_wheel_views();
    // Vehicle setup runs in _ready, which may follow this sibling's _ready.
    // Cache the topology once it becomes available without recounting wheels
    // every frame.
    if (!wheel_topology_cached && !axle_views.empty())
        cache_wheel_views();
    if (!wheel_topology_cached)
        return;

    int64_t wheel_index = 0;
    const int64_t wheel_capacity = std::min<int64_t>(
        wheel_angular_velocities.size(), tire_forces.size());
    for (const Axle *axle : axle_views) {
        for (const Wheel* w : axle->get_wheels()) {
            if (wheel_index >= wheel_capacity)
                return;
            wheel_angular_velocities.set(wheel_index, w->get_angular_velocity() * ang_to_rpm);
            tire_forces.set(wheel_index, w->get_tire_force());
            ++wheel_index;
        }
    }
}

void VehicleTelemetry::set_target(Vehicle* p_vehicle) {
    target = p_vehicle;
    cache_wheel_views();
}

Vehicle* VehicleTelemetry::get_target() const {
    return target;
}

void VehicleTelemetry::cache_wheel_views() {
    if (target == nullptr) {
        wheel_angular_velocities.resize(0);
        tire_forces.resize(0);
        cached_wheel_count = 0;
        wheel_topology_cached = false;
        return;
    }

    int wheel_count = 0;
    for (const Axle *axle : target->get_wheel_views())
        wheel_count += static_cast<int>(axle->get_wheels().size());

    if (wheel_count <= 0) {
        wheel_angular_velocities.resize(0);
        tire_forces.resize(0);
        cached_wheel_count = 0;
        wheel_topology_cached = false;
        return;
    }

    wheel_angular_velocities.resize(wheel_count);
    tire_forces.resize(wheel_count);
    cached_wheel_count = wheel_count;
    wheel_topology_cached = true;
}

}
