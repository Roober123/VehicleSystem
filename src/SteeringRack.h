#pragma once

#include <godot_cpp/core/defs.hpp>
#include <vector>
#include "godot_cpp/classes/ref.hpp"
#include "Resources/steering_rack_data.h"

namespace godot {


class SteeringRack {

    real_t angle = 0.0; // rad
    real_t angular_velocity = 0.0;
    real_t inertia = 1.0;

    real_t damping = 0.0;
    real_t friction_coefficient = 0.0;
    real_t max_angle = 0.0; // rad

    real_t proportional_gain = 0.0;
    real_t derivative_gain = 0.0;

    real_t sat_gain = 0.0;
    bool configured = false;
    public:
    void load(const Ref<SteeringRackData>& s);
    void solve(real_t steer_input, real_t sat_torque, real_t dt, real_t speed_kph = 0.0);
    real_t get_angle() const;
    bool is_configured() const { return configured; }
};

}
