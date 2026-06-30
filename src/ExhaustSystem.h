#pragma once
#include <godot_cpp/core/defs.hpp>
#include <algorithm>

class Turbo;

namespace godot {

class VehicleEngine;
class GearboxModule;

struct ExhaustData {
    real_t smoke = 0.0; // 0-1 smoke intensity
    real_t heat = 0.0; // 0-1 heat intensity
    real_t flame_probability = 0.0; // 0-1 flame burst probability
};

class ExhaustSystem {
    real_t smoke_intensity = 1.0;
    real_t heat_intensity = 1.0;
    real_t flame_intensity = 1.0;

    static constexpr real_t HEAT_SMOOTH_RATE = 2.0;
    static constexpr real_t TURBO_BUILD_RATE = 0.8;
    static constexpr real_t TURBO_DECAY_RATE = 1.5;
    static constexpr real_t THROTTLE_SMOOTH_RATE = 3.0;

    real_t smoothed_heat = 0.0;
    real_t turbo_energy = 0.0;
    real_t smoothed_throttle = 0.0;

    ExhaustData data;
    void _compute_smoke(real_t effective_fuel_flow, real_t rpm, real_t redline_rpm);
    void _compute_heat(real_t effective_fuel_flow, real_t dt, real_t turbo_boost);
    void _update_turbo_energy(real_t turbo_boost, real_t throttle, real_t dt);
    void _compute_flame_probability(real_t rpm, real_t redline_rpm, real_t throttle, bool is_shifting);

public:
    ExhaustSystem() = default;
    
    void update(real_t dt,
                VehicleEngine &engine,
                GearboxModule &gearbox,
                Turbo &turbo);

    const ExhaustData &get_data() const { return data; }

    void set_smoke_intensity(real_t v)  { smoke_intensity = std::clamp(v, real_t{0.0}, real_t{10.0}); }
    void set_heat_intensity(real_t v)   { heat_intensity = std::clamp(v, real_t{0.0}, real_t{10.0}); }
    void set_flame_intensity(real_t v)  { flame_intensity = std::clamp(v, real_t{0.0}, real_t{10.0}); }

    real_t get_smoke_intensity() const  { return smoke_intensity; }
    real_t get_heat_intensity() const   { return heat_intensity; }
    real_t get_flame_intensity() const  { return flame_intensity; }
};

}
