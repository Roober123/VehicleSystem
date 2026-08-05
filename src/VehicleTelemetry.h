#pragma once
#include "godot_cpp/classes/node.hpp"
#include <godot_cpp/core/class_db.hpp>
#include "vehicle.h"

namespace godot {

class VehicleTelemetry : public Node {
    GDCLASS(VehicleTelemetry, Node);

    Vehicle* target = nullptr;
    VehicleTelemetrySnapshot snapshot;
    int cached_wheel_count = 0;
    bool wheel_topology_cached = false;

    void cache_wheel_views();
    PackedFloat64Array wheel_angular_velocities;
    PackedVector3Array tire_forces;


protected:
    static void _bind_methods();

public:
    VehicleTelemetry() = default;
    ~VehicleTelemetry() override = default;

    void _ready() override;
    void _process(double delta) override;

    void set_target(Vehicle* p_vehicle);
    Vehicle* get_target() const;

    // Read-only getters for GDScript
    real_t get_engine_rpm() const { return snapshot.engine_rpm; }
    real_t get_engine_torque() const { return snapshot.engine_torque; }
    real_t get_engine_throttle() const { return snapshot.engine_throttle; }
    real_t get_driveshaft_rpm() const { return snapshot.driveshaft_rpm; }
    real_t get_clutch_engagement() const { return snapshot.clutch_engagement; }
    real_t get_clutch_requested_torque() const { return snapshot.clutch_requested_torque; }
    real_t get_clutch_transmitted_torque() const { return snapshot.clutch_transmitted_torque; }
    real_t get_clutch_output_torque() const { return snapshot.clutch_output_torque; }
    real_t get_clutch_slip() const { return snapshot.clutch_slip; }
    bool get_clutch_slipping() const { return snapshot.clutch_slipping; }
    int get_current_gear() const { return snapshot.current_gear; }
    real_t get_gear_ratio() const { return snapshot.gear_ratio; }
    real_t get_vehicle_speed_kph() const { return snapshot.vehicle_speed_kph; }
    real_t get_turbo_boost() const { return snapshot.turbo_boost; }
    PackedFloat64Array get_wheel_angular_velocities() const { return wheel_angular_velocities; }
    PackedVector3Array get_tire_forces() const { return tire_forces; }
};

} // namespace godot
