#pragma once
#include "godot_cpp/classes/node.hpp"
#include <godot_cpp/core/class_db.hpp>
#include "vehicle.h"

namespace godot {

class VehicleTelemetry : public Node {
    GDCLASS(VehicleTelemetry, Node);

    Vehicle* target = nullptr;

    real_t engine_rpm = 0.0;
    real_t engine_torque = 0.0;
    real_t engine_throttle = 0.0;
    real_t driveshaft_rpm = 0.0;
    real_t clutch_engagement = 0.0;
    int current_gear = 0;
    real_t gear_ratio = 0.0;
    real_t vehicle_speed_kph = 0.0;
    real_t turbo_boost = 0.0;
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
    real_t get_engine_rpm() const { return engine_rpm; }
    real_t get_engine_torque() const { return engine_torque; }
    real_t get_engine_throttle() const { return engine_throttle; }
    real_t get_driveshaft_rpm() const { return driveshaft_rpm; }
    real_t get_clutch_engagement() const { return clutch_engagement; }
    int get_current_gear() const { return current_gear; }
    real_t get_gear_ratio() const { return gear_ratio; }
    real_t get_vehicle_speed_kph() const { return vehicle_speed_kph; }
    real_t get_turbo_boost() const { return turbo_boost; }
    PackedFloat64Array get_wheel_angular_velocities() const { return wheel_angular_velocities; }
    PackedVector3Array get_tire_forces() const { return tire_forces; }
};

} // namespace godot