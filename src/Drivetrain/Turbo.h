#pragma once
#include "godot_cpp/classes/node3d.hpp"
#include "Resources/turbo_data.h"

class Turbo {
    real_t spool_up = 1.5;
    real_t spool_down = 2.5;
    real_t max_boost = 1; 

    real_t boost = 0.0; // bar

    real_t start_rpm = 2000;
    real_t max_boost_rpm = 2800;
    real_t fall_rpm = 6000;
    public:
    void configure(const godot::Ref<godot::TurboData>& data);
    real_t update(real_t dt, real_t engine_rpm, real_t throttle);
    real_t get_boost() { return boost; }
};