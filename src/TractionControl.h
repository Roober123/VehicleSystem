#pragma once

#include "axle.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace godot {

class TractionControl {
    static constexpr real_t target_slip = real_t{0.16};
    static constexpr real_t min_speed = real_t{5.0}; // m/s

public:
    real_t apply(real_t driver_throttle, real_t speed, const std::vector<Axle*>& axles);
};

}
