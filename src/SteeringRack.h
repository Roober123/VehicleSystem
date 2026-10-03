#pragma once

#include <godot_cpp/core/defs.hpp>
#include "godot_cpp/classes/ref.hpp"
#include "Resources/steering_rack_data.h"

namespace godot {


class SteeringRack {

    real_t angle = 0.0; // rad
    real_t angular_velocity = 0.0;
    real_t response_frequency = 0.0;
    real_t friction_torque = 0.0;
    real_t stiffness = 0.0;
    real_t steering_half_speed_kph = 50.0;
    real_t max_angle = 0.0; // rad

    real_t sat_gain = 0.0;
    real_t max_feedback_deflection = 0.0;
    real_t minimum_driver_authority = 0.65;
    real_t target = 0.0;
    real_t raw_feedback = 0.0;
    real_t applied_feedback = 0.0;
    void integrate_response(real_t target_angle, real_t external_torque, real_t dt);
    public:
    void load(const Ref<SteeringRackData>& s);
    void solve(real_t steer_input, real_t sat_torque, real_t dt, real_t speed_kph = 0.0);
    real_t get_angle() const;
    real_t get_target() const { return target; }
    real_t get_raw_feedback() const { return raw_feedback; }
    real_t get_applied_feedback() const { return applied_feedback; }
};

}
