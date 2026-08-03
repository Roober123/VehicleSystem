#include "VehicleDifferential.h"

const array<float, 4>& Open_Differential::update(const vector<godot::Wheel*>& wheels) {
    torque_distribution.fill(0.0f);

    size_t n = std::min(wheels.size(), size_t(4));
    if (n == 0) return torque_distribution;

    float equal_share = 1.0f / static_cast<float>(n);
    for (size_t i = 0; i < n; ++i)
        torque_distribution[i] = equal_share;

    return torque_distribution;
}
