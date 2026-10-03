#include "VehicleEngineReaction.h"

#include <algorithm>
#include <cmath>

namespace godot {

void VehicleEngineReaction::configure(const Ref<EngineReactionData> &data) {
    reset();
    enabled = data.is_valid() && data->get_enabled();
    reaction_strength = data.is_valid() ? data->get_reaction_strength() : real_t{0.0};
    vibration_strength = data.is_valid() ? data->get_vibration_strength() : real_t{0.0};
    idle_vibration_max_rpm = data.is_valid() ? data->get_idle_vibration_max_rpm() : real_t{1000.0};
    idle_vibration_multiplier = data.is_valid() ? data->get_idle_vibration_multiplier() : real_t{1.5};
    idle_frequency = data.is_valid() ? data->get_idle_vibration_frequency() : real_t{0.0};
    redline_frequency = data.is_valid() ? data->get_redline_vibration_frequency() : real_t{0.0};
    maximum_torque = data.is_valid() ? data->get_maximum_torque() : real_t{0.0};
    speed_fade_start_kph = data.is_valid() ? data->get_speed_fade_start_kph() : real_t{5.0};
    speed_fade_end_kph = data.is_valid() ? data->get_speed_fade_end_kph() : real_t{30.0};
}

EngineReactionSample VehicleEngineReaction::update(real_t generated_torque,
        real_t self_torque, real_t rpm, real_t idle_rpm, real_t redline_rpm,
        real_t sub_dt, real_t physics_dt, real_t speed_kph) {
    EngineReactionSample sample;
    if (!enabled || !std::isfinite(generated_torque) || !std::isfinite(self_torque) ||
            !std::isfinite(rpm) || !std::isfinite(idle_rpm) || !std::isfinite(speed_kph) ||
            !std::isfinite(redline_rpm) || redline_rpm <= idle_rpm ||
            !std::isfinite(sub_dt) || sub_dt <= real_t{0.0} ||
            !std::isfinite(physics_dt) || physics_dt <= real_t{0.0} || sub_dt > physics_dt)
        return sample;

    const real_t fade_width = std::max(speed_fade_end_kph - speed_fade_start_kph, real_t{1.0});
    const real_t fade_position = std::clamp((std::abs(speed_kph) - speed_fade_start_kph) / fade_width,
            real_t{0.0}, real_t{1.0});
    // Smoothstep gives zero slope at both endpoints. Fade the complete engine
    // effect so even a strong steady reaction cannot steer the chassis at speed.
    sample.speed_factor = real_t{1.0} - fade_position * fade_position *
            (real_t{3.0} - real_t{2.0} * fade_position);

    const real_t rpm_fraction = std::clamp((rpm - idle_rpm) / (redline_rpm - idle_rpm),
            real_t{0.0}, real_t{1.0});
    // Chassis physics advances once per tick, regardless of drivetrain substeps.
    // Preserve at least five chassis samples per rumble cycle to avoid aliasing.
    sample.vibration_frequency = std::min(
            (real_t{1.0} - rpm_fraction) * idle_frequency + rpm_fraction * redline_frequency,
            real_t{0.2} / physics_dt);
    sample.reaction_torque = std::clamp(-reaction_strength * self_torque,
            -maximum_torque, maximum_torque);
    // Reserve symmetric headroom instead of clipping the sine, which could
    // otherwise introduce a DC bias into vibration under sustained load.
    const real_t idle_gain = rpm <= idle_vibration_max_rpm ? idle_vibration_multiplier : real_t{1.0};
    sample.vibration_amplitude = std::min(
            vibration_strength * std::max(generated_torque, real_t{0.0}) * idle_gain,
            std::max(maximum_torque - std::abs(sample.reaction_torque), real_t{0.0}));
    // Fade after bounding: releasing cap headroom at speed must not make the
    // oscillating component larger while the overall effect is fading out.
    sample.reaction_torque *= sample.speed_factor;
    sample.vibration_amplitude *= sample.speed_factor;
    if (sample.vibration_frequency == real_t{0.0}) {
        // A stopped oscillator must not apply a sustained torque at its last phase.
        sample.vibration_amplitude = 0.0;
        return sample;
    }

    constexpr double two_pi = 6.28318530717958647692;
    const double half_step = two_pi * double(sample.vibration_frequency) * double(sub_dt) * 0.5;
    const double sinc = half_step > 1e-12 ? std::sin(half_step) / half_step : 1.0;
    // Exact average of the sine over this substep for constant amplitude/frequency.
    sample.vibration_torque = sample.vibration_amplitude * std::sin(phase + half_step) * sinc;
    phase = std::fmod(phase + 2.0 * half_step, two_pi);
    return sample;
}

} // namespace godot
