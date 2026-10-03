#pragma once

#include "godot_cpp/classes/resource.hpp"

namespace godot {

/// Optional chassis reaction and authored rumble, copied at setup/restart.
class EngineReactionData : public Resource {
    GDCLASS(EngineReactionData, Resource);

    bool enabled = true;
    real_t reaction_strength = 0.25;
    real_t vibration_strength = 0.3;
    real_t idle_vibration_max_rpm = 1000.0;
    real_t idle_vibration_multiplier = 1.5;
    real_t idle_vibration_frequency = 5.0;
    real_t redline_vibration_frequency = 20.0;
    real_t maximum_torque = 250.0;
    real_t speed_fade_start_kph = 5.0;
    real_t speed_fade_end_kph = 30.0;

protected:
    static void _bind_methods();

public:
    void set_enabled(bool value) { enabled = value; }
    bool get_enabled() const { return enabled; }
    void set_reaction_strength(real_t value);
    real_t get_reaction_strength() const { return reaction_strength; }
    void set_vibration_strength(real_t value);
    real_t get_vibration_strength() const { return vibration_strength; }
    void set_idle_vibration_max_rpm(real_t value);
    real_t get_idle_vibration_max_rpm() const { return idle_vibration_max_rpm; }
    void set_idle_vibration_multiplier(real_t value);
    real_t get_idle_vibration_multiplier() const { return idle_vibration_multiplier; }
    void set_idle_vibration_frequency(real_t value);
    real_t get_idle_vibration_frequency() const { return idle_vibration_frequency; }
    void set_redline_vibration_frequency(real_t value);
    real_t get_redline_vibration_frequency() const { return redline_vibration_frequency; }
    void set_maximum_torque(real_t value);
    real_t get_maximum_torque() const { return maximum_torque; }
    void set_speed_fade_start_kph(real_t value);
    real_t get_speed_fade_start_kph() const { return speed_fade_start_kph; }
    void set_speed_fade_end_kph(real_t value);
    real_t get_speed_fade_end_kph() const { return speed_fade_end_kph; }
};

} // namespace godot
