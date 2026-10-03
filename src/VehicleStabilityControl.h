#pragma once

#include <vector>

#include "godot_cpp/variant/basis.hpp"
#include "Resources/esc_data.h"

namespace godot {

class Axle;

struct StabilityState {
    Basis body_basis;
    Vector3 linear_velocity;
    Vector3 angular_velocity;
    real_t vehicle_mass = 0.0;
    real_t wheelbase = 0.0;
    real_t trackwidth = 0.0;
    real_t steer_angle = 0.0;
    int grounded_wheels = 0;
};

struct StabilityTelemetry {
    real_t yaw_target = 0.0;
    real_t yaw_error = 0.0;
    real_t requested_torque = 0.0; // before the torque cap
    real_t applied_torque = 0.0;
    bool cap_active = false;
    bool slew_active = false;
    bool speed_gate = false;
    bool supported = false;
};

/// Optional chassis yaw controller, value-owned by Vehicle.
class VehicleStabilityControl {
    Ref<ESCData> resource;
    bool smoothing_enabled = false;
    real_t engagement_rate = 60000.0;
    real_t release_rate = 120000.0;
    real_t applied_torque = 0.0;
    StabilityTelemetry telemetry;
    bool valid_state(const StabilityState &state) const;
    Vector3 raw_torque(const StabilityState &state, StabilityTelemetry *sample) const;
    bool enabled = false;
    real_t yaw_damping = 2.0;
    real_t minimum_speed = 3.0;
    real_t maximum_corrective_torque = 6000.0;
    real_t maximum_target_lateral_acceleration = 9.81;

public:
    void load_parameters(const Ref<ESCData> &data);
    StabilityState get_state(const Basis &body_basis,
                             const Vector3 &linear_velocity,
                             const Vector3 &angular_velocity,
                             real_t vehicle_mass,
                             const std::vector<Axle *> &axles) const;
    Vector3 update(const StabilityState &state, real_t dt);
    void reset();
    const StabilityTelemetry &get_telemetry() const { return telemetry; }
    Vector3 compute_torque(const StabilityState &state) const;
};

} // namespace godot
