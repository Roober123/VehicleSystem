#pragma once

#include <godot_cpp/core/defs.hpp>
#include <vector>

namespace godot {


class SteeringRack {

    real_t angle; // rad
    real_t angular_velocity;
    real_t inertia;

    real_t damping;
    real_t friction_coefficient;
    real_t max_angle; // rad

    real_t proportional_gain;
    real_t derivative_gain;

    real_t sat_gain;
    
    void solve(real_t steer_input, real_t sat_torque);

};

}