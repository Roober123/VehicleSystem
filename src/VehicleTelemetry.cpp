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
    ClassDB::bind_method(D_METHOD("get_clutch_requested_torque"), &VehicleTelemetry::get_clutch_requested_torque);
    ClassDB::bind_method(D_METHOD("get_clutch_transmitted_torque"), &VehicleTelemetry::get_clutch_transmitted_torque);
    ClassDB::bind_method(D_METHOD("get_clutch_output_torque"), &VehicleTelemetry::get_clutch_output_torque);
    ClassDB::bind_method(D_METHOD("get_clutch_slip"), &VehicleTelemetry::get_clutch_slip);
    ClassDB::bind_method(D_METHOD("get_clutch_slipping"), &VehicleTelemetry::get_clutch_slipping);
    ClassDB::bind_method(D_METHOD("get_current_gear"), &VehicleTelemetry::get_current_gear);
    ClassDB::bind_method(D_METHOD("get_gear_ratio"), &VehicleTelemetry::get_gear_ratio);
    ClassDB::bind_method(D_METHOD("get_vehicle_speed_kph"), &VehicleTelemetry::get_vehicle_speed_kph);
    ClassDB::bind_method(D_METHOD("get_turbo_boost"), &VehicleTelemetry::get_turbo_boost);
    ClassDB::bind_method(D_METHOD("get_wheel_rpms"), &VehicleTelemetry::get_wheel_rpms);
    ClassDB::bind_method(D_METHOD("get_tire_forces"), &VehicleTelemetry::get_tire_forces);
    ClassDB::bind_method(D_METHOD("get_wheel_rpm", "wheel"),
                         &VehicleTelemetry::get_wheel_rpm);
    ClassDB::bind_method(D_METHOD("get_tire_force", "wheel"),
                         &VehicleTelemetry::get_tire_force);
    ClassDB::bind_method(D_METHOD("get_tire_telemetry", "wheel"),
                         &VehicleTelemetry::get_tire_telemetry);
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
        wheel_rpms.size(), tire_forces.size());
    for (const Axle *axle : axle_views) {
        for (const Wheel* w : axle->get_wheels()) {
            if (wheel_index >= wheel_capacity)
                return;
            wheel_rpms.set(wheel_index, w->get_angular_velocity() * ang_to_rpm);
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

real_t VehicleTelemetry::get_wheel_rpm(Wheel *wheel) const {
    if (wheel == nullptr)
        return real_t{0.0};
    constexpr real_t ang_to_rpm = real_t{60.0} / (real_t{2.0} * Math_PI);
    return wheel->get_angular_velocity() * ang_to_rpm;
}

Vector3 VehicleTelemetry::get_tire_force(Wheel *wheel) const {
    return wheel != nullptr ? wheel->get_tire_force() : Vector3();
}

Dictionary VehicleTelemetry::get_tire_telemetry(Wheel *wheel) const {
    Dictionary data;
    if (wheel == nullptr)
        return data;

    data["wheel"] = wheel;
    data["wheel_rpm"] = get_wheel_rpm(wheel);
    data["tire_force"] = get_tire_force(wheel);
    data["normal_load"] = wheel->get_suspension_rebound_force();
    data["slip_ratio"] = wheel->get_slip_ratio();
    data["slip_angle_degrees"] = wheel->get_slip_angle();
    data["grounded"] = wheel->is_on_ground();
    data["sliding"] = wheel->get_is_sliding();
    data["abs_active"] = wheel->get_abs_active();
    data["contact_position"] = wheel->get_collision_point();
    data["contact_normal"] = wheel->get_collision_normal();
    data["grip_multiplier"] = wheel->get_grip_multiplier();
    return data;
}

void VehicleTelemetry::cache_wheel_views() {
    if (target == nullptr) {
        wheel_rpms.resize(0);
        tire_forces.resize(0);
        cached_wheel_count = 0;
        wheel_topology_cached = false;
        return;
    }

    int wheel_count = 0;
    for (const Axle *axle : target->get_wheel_views())
        wheel_count += static_cast<int>(axle->get_wheels().size());

    if (wheel_count <= 0) {
        wheel_rpms.resize(0);
        tire_forces.resize(0);
        cached_wheel_count = 0;
        wheel_topology_cached = false;
        return;
    }

    wheel_rpms.resize(wheel_count);
    tire_forces.resize(wheel_count);
    cached_wheel_count = wheel_count;
    wheel_topology_cached = true;
}

}
