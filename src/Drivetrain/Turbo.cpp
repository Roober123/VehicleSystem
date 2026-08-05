#include "Turbo.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr godot::real_t kBoostRiseSeconds = godot::real_t{0.08};
constexpr godot::real_t kBoostVentSeconds = godot::real_t{0.04};
constexpr godot::real_t kLiftThrottleThreshold = godot::real_t{0.01};

godot::real_t approach_exponential(godot::real_t current,
                                   godot::real_t target,
                                   godot::real_t dt,
                                   godot::real_t time_constant) {
    return current + (target - current) *
            (godot::real_t{1.0} - std::exp(-dt / time_constant));
}

} // namespace

void Turbo::configure(const godot::Ref<godot::TurboData> &data) {
    shaft_energy = godot::real_t{0.0};
    boost_bar = godot::real_t{0.0};
    max_boost_bar = data->get_max_boost_bar();
    full_boost_rpm = data->get_full_boost_rpm();
    lag_seconds = data->get_lag_seconds();
}

godot::real_t Turbo::update(godot::real_t dt, godot::real_t engine_rpm,
                            godot::real_t effective_throttle,
                            godot::real_t normalized_base_torque) {
    const godot::real_t rpm_ratio = std::clamp(engine_rpm / full_boost_rpm,
                                               godot::real_t{0.0},
                                               godot::real_t{1.0});
    // Engine load shapes spool below the authored full-boost point, but must
    // not lower that point itself. At full throttle and full_boost_rpm the
    // exhaust target is therefore exactly 1, regardless of torque-curve shape.
    const godot::real_t load_factor =
            normalized_base_torque +
            (godot::real_t{1.0} - normalized_base_torque) * rpm_ratio;
    const godot::real_t exhaust_target =
            effective_throttle * load_factor * rpm_ratio;

    const godot::real_t shaft_tau = exhaust_target >= shaft_energy ? lag_seconds : lag_seconds * godot::real_t{2.0};
    shaft_energy = approach_exponential(shaft_energy, exhaust_target,
                                        dt, shaft_tau);

    const godot::real_t compressor_target = max_boost_bar * shaft_energy * shaft_energy;
    const bool on_throttle = effective_throttle > kLiftThrottleThreshold;
    const godot::real_t boost_target = on_throttle ? compressor_target : godot::real_t{0.0};
    const godot::real_t boost_tau = on_throttle ? kBoostRiseSeconds : kBoostVentSeconds;
    boost_bar = approach_exponential(boost_bar, boost_target, dt, boost_tau);
    return boost_bar;
}

godot::real_t Turbo::get_air_charge_ratio() const {
    return godot::real_t{1.0} + boost_bar * godot::real_t{0.85};
}
