#pragma once

#include "Resources/engine_reaction_data.h"

namespace godot {

struct EngineReactionSample {
    real_t reaction_torque = 0.0;
    real_t vibration_torque = 0.0;
    real_t vibration_amplitude = 0.0;
    real_t vibration_frequency = 0.0;
    real_t speed_factor = 0.0;
    real_t total_torque() const { return reaction_torque + vibration_torque; }
};

/// Produces substep-averaged torque. No rigid-body mutation or allocation.
class VehicleEngineReaction {
    bool enabled = false;
    real_t reaction_strength = 0.0;
    real_t vibration_strength = 0.0;
    real_t idle_vibration_max_rpm = 1000.0;
    real_t idle_vibration_multiplier = 1.5;
    real_t idle_frequency = 0.0;
    real_t redline_frequency = 0.0;
    real_t maximum_torque = 0.0;
    real_t speed_fade_start_kph = 5.0;
    real_t speed_fade_end_kph = 30.0;
    double phase = 0.0;

public:
    void configure(const Ref<EngineReactionData> &data);
    void reset() { phase = 0.0; }
    EngineReactionSample update(real_t generated_torque, real_t self_torque,
            real_t rpm, real_t idle_rpm, real_t redline_rpm,
            real_t sub_dt, real_t physics_dt, real_t speed_kph = 0.0);
};

} // namespace godot
