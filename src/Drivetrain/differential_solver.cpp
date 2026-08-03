#include "differential_solver.h"

#include <algorithm>
#include <cmath>

namespace godot {

DifferentialSolver::Snapshot::Snapshot(const DifferentialData &data) :
        mode(data.get_mode()),
        preload_torque(data.get_preload_torque()),
        power_lock_ratio(data.get_power_lock_ratio()),
        coast_lock_ratio(data.get_coast_lock_ratio()),
        slip_sensitive_gain(data.get_slip_sensitive_gain()),
        max_lock_torque(data.get_max_lock_torque()) {}

real_t DifferentialSolver::solve(const Input &input) const {
    if ((snapshot.mode != DifferentialData::OPEN &&
         snapshot.mode != DifferentialData::LIMITED_SLIP &&
         snapshot.mode != DifferentialData::LOCKED) ||
            !std::isfinite(snapshot.preload_torque) ||
            !std::isfinite(snapshot.power_lock_ratio) ||
            !std::isfinite(snapshot.coast_lock_ratio) ||
            !std::isfinite(snapshot.slip_sensitive_gain) ||
            !std::isfinite(snapshot.max_lock_torque) ||
            !std::isfinite(input.left_inertia) ||
            !std::isfinite(input.right_inertia) ||
            !std::isfinite(input.left_free_velocity) ||
            !std::isfinite(input.right_free_velocity) ||
            !std::isfinite(input.transmitted_torque) ||
            !std::isfinite(input.carrier_velocity) ||
            !std::isfinite(input.dt) || input.left_inertia <= real_t{0.0} ||
            input.right_inertia <= real_t{0.0} || input.dt <= real_t{0.0})
        return real_t{0.0};

    if (snapshot.mode == DifferentialData::OPEN)
        return real_t{0.0};

    const real_t relative_velocity =
            input.left_free_velocity - input.right_free_velocity;
    if (!std::isfinite(relative_velocity) || relative_velocity == real_t{0.0})
        return real_t{0.0};

    const real_t inverse_inertia_sum =
            real_t{1.0} / input.left_inertia + real_t{1.0} / input.right_inertia;
    if (!std::isfinite(inverse_inertia_sum) || inverse_inertia_sum <= real_t{0.0})
        return real_t{0.0};
    const real_t relative_inertia = real_t{1.0} / inverse_inertia_sum;
    if (!std::isfinite(relative_inertia) || relative_inertia <= real_t{0.0})
        return real_t{0.0};

    const real_t ideal_impulse = -relative_inertia * relative_velocity;
    if (!std::isfinite(ideal_impulse))
        return real_t{0.0};
    if (snapshot.mode == DifferentialData::LOCKED)
        return ideal_impulse;

    const real_t torque_direction = input.transmitted_torque * input.carrier_velocity;
    if (!std::isfinite(torque_direction))
        return real_t{0.0};
    const real_t lock_ratio = torque_direction >= real_t{0.0}
            ? snapshot.power_lock_ratio : snapshot.coast_lock_ratio;
    const real_t preload = std::max(snapshot.preload_torque, real_t{0.0});
    const real_t nonnegative_ratio = std::max(lock_ratio, real_t{0.0});
    const real_t nonnegative_gain = std::max(snapshot.slip_sensitive_gain, real_t{0.0});
    const real_t configured_max = std::max(snapshot.max_lock_torque, real_t{0.0});
    const real_t capacity = std::min(
            configured_max,
            preload + nonnegative_ratio * std::abs(input.transmitted_torque) +
                    nonnegative_gain * std::abs(relative_velocity));
    if (!std::isfinite(capacity) || capacity <= real_t{0.0})
        return real_t{0.0};

    const real_t impulse_limit = capacity * input.dt;
    if (!std::isfinite(impulse_limit) || impulse_limit <= real_t{0.0})
        return real_t{0.0};

    // The ideal impulse already points toward zero relative speed.  Limiting
    // its magnitude cannot cross that target in this step.
    return ideal_impulse < real_t{0.0}
            ? -std::min(-ideal_impulse, impulse_limit)
            : std::min(ideal_impulse, impulse_limit);
}

} // namespace godot
