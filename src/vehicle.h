#pragma once

#include <vector>

#include "godot_cpp/classes/physics_direct_body_state3d.hpp"
#include "godot_cpp/classes/rigid_body3d.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/property_info.hpp>

#include "Resources/vehicle_config.h"
#include "VehicleDrivetrain.h"
#include "VehicleRunningGear.h"

namespace godot {

/// Immutable-by-value scalar view used by telemetry consumers. It deliberately
/// contains no references into the live drivetrain or running gear.
struct VehicleTelemetrySnapshot {
    real_t engine_rpm = 0.0;
    real_t engine_torque = 0.0;
    real_t engine_throttle = 0.0;
    real_t driveshaft_rpm = 0.0;
    real_t clutch_engagement = 0.0;
    real_t clutch_requested_torque = 0.0;
    real_t clutch_transmitted_torque = 0.0;
    real_t clutch_output_torque = 0.0;
    real_t clutch_slip = 0.0;
    bool clutch_slipping = false;
    int current_gear = 0;
    real_t gear_ratio = 0.0;
    real_t vehicle_speed_kph = 0.0;
    real_t turbo_boost = 0.0;
};

class Vehicle : public RigidBody3D {
    GDCLASS(Vehicle, RigidBody3D);

    VehicleRunningGear running_gear;
    VehicleDrivetrain drivetrain;
    Ref<VehicleConfig> config = nullptr;

    real_t throttle_input = 0.0;
    real_t steer_input = 0.0;
    real_t brake_input = 0.0;
    bool abs_enabled = true;
    bool tcs_enabled = true;
    int substeps = 1;
    bool initialized = false;

protected:
    static void _bind_methods();

public:
    Vehicle();
    ~Vehicle() override = default;

    virtual void _ready() override;
    virtual void _integrate_forces(PhysicsDirectBodyState3D *state) override;

    void set_config(const Ref<VehicleConfig> &value);
    Ref<VehicleConfig> get_config() const { return config; }

    void set_throttle_input(real_t value);
    real_t get_throttle_input() const { return throttle_input; }
    void set_steer_input(real_t value);
    real_t get_steer_input() const { return steer_input; }
    void set_brake_input(real_t value);
    real_t get_brake_input() const { return brake_input; }
    void set_abs_enabled(bool value) { abs_enabled = value; }
    bool get_abs_enabled() const { return abs_enabled; }
    void set_tcs_enabled(bool value);
    bool get_tcs_enabled() const { return tcs_enabled; }

    void shift_up();
    void shift_down();
    void select_drive();
    void select_neutral();
    void select_reverse();

    void set_substeps(int value);
    int get_substeps() const { return substeps; }

    VehicleTelemetrySnapshot get_telemetry_snapshot() const;
    const std::vector<Axle *> &get_wheel_views() const {
        return running_gear.get_axles();
    }
    real_t get_speed_kph() const;
};

} // namespace godot
