#include "TractionControl.h"

namespace godot {

real_t TractionControl::apply(real_t driver_throttle, real_t speed,
                              const std::vector<Axle*>& axles) {
    if (speed < min_speed || driver_throttle < real_t{0.05}) {
        return driver_throttle;
    }

    real_t max_slip = real_t{0.0};
    for (const auto* ax : axles) {
        if (ax->drive_ratio <= real_t{0.0}) continue;
        for (const auto* wh : ax->get_wheels())
            max_slip = std::max(max_slip, std::abs(wh->get_slip_ratio()));
    }
    constexpr real_t cutoff_range = real_t{0.5}; 
    real_t excess = max_slip - target_slip;
    real_t cut = real_t{0.0};
    if (excess > real_t{0.0}) {
        cut = std::min(excess / cutoff_range, real_t{1.0});
    }

    return driver_throttle * (real_t{1.0} - cut);
}

}
