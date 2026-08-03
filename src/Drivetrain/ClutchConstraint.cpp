#include "ClutchConstraint.h"

#include <algorithm>

namespace godot {

void ClutchConstraint::solve(real_t dt, const ClutchSolveInput &input) {
    if (!std::isfinite(dt) || dt <= real_t{0.0})
        return;

    const real_t ratio = input.ratio;
    if (std::abs(ratio) <= real_t{1e-8})
        return;

    const real_t engine_inertia = engine->get_inertia();
    const real_t output_inertia = input.aggregate_inertia;

    const real_t engine_angular_velocity = engine->get_angular_velocity();
    const real_t output_angular_velocity = input.aggregate_angular_velocity;

    const real_t reflected_inertia = output_inertia / (ratio * ratio);
    const real_t effective_inertia =
            (engine_inertia * reflected_inertia) /
            (engine_inertia + reflected_inertia);
    if (!std::isfinite(reflected_inertia) || !std::isfinite(effective_inertia) ||
            reflected_inertia <= real_t{0.0} || effective_inertia <= real_t{0.0})
        return;

    const real_t current_slip = engine_angular_velocity - ratio * output_angular_velocity;
    const real_t free_slip = current_slip + dt * (input.pending_engine_torque / engine_inertia);
    if (!std::isfinite(current_slip) || !std::isfinite(free_slip))
        return;

    real_t clutch_impulse = effective_inertia * free_slip;
    const real_t engagement = std::max(input.engagement, real_t{0.0});
    const real_t capacity = std::max(input.capacity, real_t{0.0});
    const real_t max_impulse = engagement * capacity * dt;
    if (!std::isfinite(clutch_impulse) || !std::isfinite(max_impulse))
        return;

    clutch_impulse = std::clamp(clutch_impulse, -max_impulse, max_impulse);
    // Preserve the established conservative no-crossing behavior for nonzero
    // slip while leaving synchronized holding torque exact.
    if (std::abs(current_slip) > real_t{0.0} && clutch_impulse != real_t{0.0})
        clutch_impulse = std::nextafter(clutch_impulse, real_t{0.0});

    const real_t clutch_torque = clutch_impulse / dt;
    engine->add_torque(-clutch_torque);
    output->add_torque(clutch_torque * ratio);
}

} // namespace godot
