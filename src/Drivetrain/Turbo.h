#pragma once

#include "Resources/turbo_data.h"

class Turbo {
    // Shaft energy is normalized and always kept in [0, 1].  Boost is stored
    // in bar and bounded by the configured maximum.
    godot::real_t shaft_energy = godot::real_t{0.0};
    godot::real_t boost_bar = godot::real_t{0.0};
    godot::real_t max_boost_bar = godot::real_t{1.0};
    godot::real_t full_boost_rpm = godot::real_t{3000.0};
    godot::real_t lag_seconds = godot::real_t{0.6};

public:
    void configure(const godot::Ref<godot::TurboData> &data);

    // normalized_base_torque is the once-sampled, normalized engine curve.
    godot::real_t update(godot::real_t dt, godot::real_t engine_rpm,
                         godot::real_t effective_throttle,
                         godot::real_t normalized_base_torque = godot::real_t{1.0});

    godot::real_t get_boost() const { return boost_bar; }
    godot::real_t get_shaft_energy() const { return shaft_energy; }
    godot::real_t get_air_charge_ratio() const;
};
