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

const array<float, 4>& LSD_Differential::update(const vector<godot::Wheel*>& wheels) {
    torque_distribution.fill(0.0f);

    size_t n = std::min(wheels.size(), size_t(4));
    if (n == 0) return torque_distribution;
    if (n == 1) {
        torque_distribution[0] = 1.0f;
        return torque_distribution;
    }

    float omega_L = static_cast<float>(wheels[0]->body.get_angular_velocity());
    float omega_R = static_cast<float>(wheels[1]->body.get_angular_velocity());
    float omega_diff = omega_L - omega_R;
    float abs_diff = std::abs(omega_diff);

    //[0.286, 0.714]
    float max_bias = 0.5f * (bias_ratio - 1.0f) / (bias_ratio + 1.0f);
    float dead_zone = preload * 0.005f;

    // Beyond the dead zone, bias ramps up smoothly toward max_bias
    // using a decaying-exponential-shaped curve: t = x / (x + ramp_rate)
    // where ramp_rate controls how quickly full bias is reached.
    float bias = 0.0f;
    if (abs_diff > dead_zone) {
        float excess = abs_diff - dead_zone;
        float ramp_rate = 3.0f;  // rad/s of excess to reach ~50% of max bias
        float t = excess / (excess + ramp_rate);
        bias = t * max_bias;
        // Bias torque toward the slower wheel
        if (omega_diff > 0.0f) bias = -bias;
    }

    torque_distribution[0] = 0.5f + bias;
    torque_distribution[1] = 0.5f - bias;

    return torque_distribution;
}

const array<float, 4>& Torsen_Differential::update(const vector<godot::Wheel*>& wheels) {
    torque_distribution.fill(0.0f);

    size_t n = std::min(wheels.size(), size_t(4));
    if (n == 0) return torque_distribution;
    if (n == 1) {
        torque_distribution[0] = 1.0f;
        return torque_distribution;
    }

    float omega_L = static_cast<float>(wheels[0]->body.get_angular_velocity());
    float omega_R = static_cast<float>(wheels[1]->body.get_angular_velocity());

    float max_frac = torque_bias_ratio / (1.0f + torque_bias_ratio);
    float min_frac = 1.0f / (1.0f + torque_bias_ratio);

    if (omega_L > omega_R) {
        torque_distribution[0] = min_frac;  // faster = less torque
        torque_distribution[1] = max_frac;  // slower = more torque
    } else if (omega_R > omega_L) {
        torque_distribution[0] = max_frac;
        torque_distribution[1] = min_frac;
    } else {
        torque_distribution[0] = 0.5f;
        torque_distribution[1] = 0.5f;
    }

    return torque_distribution;
}