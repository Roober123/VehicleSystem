#include "Turbo.h"

void Turbo::configure(const godot::Ref<godot::TurboData>& data) {
    if (data.is_null()) return;
    spool_up = data->get_spool_up();
    spool_down = data->get_spool_down();
    max_boost = data->get_max_boost();
    start_rpm = data->get_start_rpm();
    max_boost_rpm = data->get_max_boost_rpm();
    fall_rpm = data->get_fall_rpm();
}

real_t Turbo::update(real_t dt, real_t engine_rpm, real_t throttle) {
    real_t target_boost = 0.0;
    if (engine_rpm > start_rpm && engine_rpm < fall_rpm) {
        if (engine_rpm > max_boost_rpm) target_boost = throttle * max_boost;
        else    target_boost = throttle * (max_boost_rpm - engine_rpm) / 
                               (max_boost_rpm - start_rpm) * max_boost;
    }
    real_t grow_rate = spool_up;
    if (target_boost < boost) grow_rate = spool_down;
    boost += (target_boost - boost) * grow_rate * dt;
    return boost;
}