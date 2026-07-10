#include "Turbo.h"
#include <algorithm>

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
    if (engine_rpm >= start_rpm && engine_rpm < fall_rpm) {
        const real_t spool_range = max_boost_rpm - start_rpm;
        real_t spool_fraction = real_t{1.0};
        if (spool_range > real_t{1e-6}) {
            spool_fraction = std::clamp((engine_rpm - start_rpm) / spool_range,
                                        real_t{0.0}, real_t{1.0});
        }
        target_boost = throttle * max_boost * spool_fraction;
    }
    real_t grow_rate = spool_up;
    if (target_boost < boost) grow_rate = spool_down;
    const real_t blend = std::clamp(grow_rate * dt, real_t{0.0}, real_t{1.0});
    boost += (target_boost - boost) * blend;
    return boost;
}
