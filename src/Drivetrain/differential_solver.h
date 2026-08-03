#pragma once

#include "Resources/differential_data.h"

namespace godot {

/// Allocation-free relative-motion solve for one two-sided differential.
///
/// The returned impulse is applied as +J to the left side and -J to the right
/// side. This preserves total angular momentum and the inertia-weighted
/// carrier velocity for the relative solve.
class DifferentialSolver {
public:
    struct Snapshot {
        DifferentialData::Mode mode = DifferentialData::OPEN;
        real_t preload_torque = 0.0;
        real_t power_lock_ratio = 0.0;
        real_t coast_lock_ratio = 0.0;
        real_t slip_sensitive_gain = 0.0;
        real_t max_lock_torque = 0.0;

        Snapshot() = default;
        explicit Snapshot(DifferentialData::Mode p_mode,
                          real_t p_preload_torque = 0.0,
                          real_t p_power_lock_ratio = 0.0,
                          real_t p_coast_lock_ratio = 0.0,
                          real_t p_slip_sensitive_gain = 0.0,
                          real_t p_max_lock_torque = 0.0) :
                mode(p_mode),
                preload_torque(p_preload_torque),
                power_lock_ratio(p_power_lock_ratio),
                coast_lock_ratio(p_coast_lock_ratio),
                slip_sensitive_gain(p_slip_sensitive_gain),
                max_lock_torque(p_max_lock_torque) {}
        explicit Snapshot(const DifferentialData &data);
    };

    struct Input {
        real_t left_inertia = 0.0;
        real_t right_inertia = 0.0;
        real_t left_free_velocity = 0.0;
        real_t right_free_velocity = 0.0;
        real_t transmitted_torque = 0.0;
        real_t carrier_velocity = 0.0;
        real_t dt = 0.0;
    };

private:
    Snapshot snapshot;

public:
    DifferentialSolver() = default;
    explicit DifferentialSolver(const Snapshot &value) : snapshot(value) {}
    explicit DifferentialSolver(const DifferentialData &data) : snapshot(data) {}

    void set_snapshot(const Snapshot &value) { snapshot = value; }
    const Snapshot &get_snapshot() const { return snapshot; }

    real_t solve(const Input &input) const;
};

} // namespace godot
