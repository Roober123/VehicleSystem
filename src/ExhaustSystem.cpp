#include "ExhaustSystem.h"
#include "Drivetrain/VehicleEngine.h"
#include "Drivetrain/GearboxModule.h"
#include "Drivetrain/Turbo.h"
#include <algorithm>

namespace godot {

void ExhaustSystem::update(real_t dt,
                           VehicleEngine &engine,
                           GearboxModule &gearbox,
                           Turbo &turbo) {
    const real_t throttle = std::clamp(engine.throttle, real_t{0.0}, real_t{1.0});
    const real_t rpm = engine.get_rpm();
    const real_t redline_rpm = engine.get_redline_rpm();
    const real_t turbo_boost = std::max(turbo.get_boost(), real_t{0.0});
    const bool is_shifting = gearbox.is_shifting();
    const int current_gear = gearbox.get_current_gear();

    const real_t rpm_norm = engine.get_rpm_normalized();

    const real_t rpm_factor = std::clamp(rpm_norm * real_t{1.5}, real_t{0.0}, real_t{1.0});
    const real_t fuel_flow = throttle * rpm_factor;

    const real_t load_factor = real_t{1.0} - rpm_norm * real_t{0.5};
    const real_t effective_fuel_flow = fuel_flow * load_factor;

    
    const real_t rate = std::clamp(THROTTLE_SMOOTH_RATE * dt, real_t{0.0}, real_t{1.0});
    smoothed_throttle += (throttle - smoothed_throttle) * rate;

    _compute_smoke(effective_fuel_flow, rpm, redline_rpm);
    _compute_heat(effective_fuel_flow, dt, turbo_boost);
    _update_turbo_energy(turbo_boost, throttle, dt);
    _compute_flame_probability(rpm, redline_rpm, throttle, is_shifting);
}

void ExhaustSystem::_compute_smoke(real_t effective_fuel_flow, real_t rpm, real_t redline_rpm) {
    const real_t smoke_rpm_mid = redline_rpm * real_t{0.4};

    real_t smoke_target = effective_fuel_flow;
    if (rpm > smoke_rpm_mid) {
        const real_t fade = (rpm - smoke_rpm_mid) / (redline_rpm - smoke_rpm_mid);
        smoke_target *= real_t{1.0} - std::clamp(fade, real_t{0.0}, real_t{1.0});
    }
    smoke_target *= smoke_intensity;
    data.smoke = std::clamp(smoke_target, real_t{0.0}, real_t{1.0});
}

void ExhaustSystem::_compute_heat(real_t effective_fuel_flow, real_t dt, real_t turbo_boost) {
    real_t heat_target = effective_fuel_flow * heat_intensity;
    heat_target = std::clamp(heat_target, real_t{0.0}, real_t{1.0});

    const real_t rate = std::clamp(HEAT_SMOOTH_RATE * dt, real_t{0.0}, real_t{1.0});
    smoothed_heat += (heat_target - smoothed_heat) * rate;

    const real_t boost_heat = turbo_boost * real_t{0.15};
    data.heat = std::clamp(smoothed_heat + boost_heat, real_t{0.0}, real_t{1.0});
}

void ExhaustSystem::_update_turbo_energy(real_t turbo_boost, real_t throttle, real_t dt) {
    if (turbo_boost > real_t{0.05} && throttle > real_t{0.1}) 
        turbo_energy += (real_t{1.0} - turbo_energy) * TURBO_BUILD_RATE * dt;
    else
        turbo_energy -= turbo_energy * TURBO_DECAY_RATE * dt;
    turbo_energy = std::clamp(turbo_energy, real_t{0.0}, real_t{1.0});
}

void ExhaustSystem::_compute_flame_probability(real_t rpm, real_t redline_rpm, real_t throttle, bool is_shifting) {
    real_t prob = real_t{0.0};

    const bool throttle_lift = (smoothed_throttle > real_t{0.2}) && (throttle < real_t{0.05});
    if (throttle_lift)
        prob += turbo_energy * real_t{3.0};
    
    if (is_shifting)
        prob += real_t{0.3};

    if (rpm >= redline_rpm)
        prob += real_t{0.6};

    prob += turbo_energy * real_t{0.375};

    data.flame_probability = std::clamp(prob * flame_intensity, real_t{0.0}, real_t{1.0});
}

}
