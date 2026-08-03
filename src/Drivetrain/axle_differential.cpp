#include "axle_differential.h"

#include <algorithm>
#include <cmath>

namespace godot {

bool AxleDifferential::configure(const Ref<DifferentialData> &data,
                                 RotationalBody *left_body,
                                 RotationalBody *right_body) {
    reset();
    if (data.is_null() || left_body == nullptr || right_body == nullptr ||
            left_body == right_body)
        return false;

    const real_t left_value = left_body->get_inertia();
    const real_t right_value = right_body->get_inertia();
    if (!std::isfinite(left_value) || left_value <= real_t{0.0} ||
            !std::isfinite(right_value) || right_value <= real_t{0.0})
        return false;
    const real_t inertia_scale = std::max({real_t{1.0}, std::abs(left_value), std::abs(right_value)});
    if (std::abs(left_value - right_value) > real_t{1e-6} * inertia_scale)
        return false;

    const real_t inverse_inertia_sum =
            real_t{1.0} / left_value + real_t{1.0} / right_value;
    if (!std::isfinite(inverse_inertia_sum) || inverse_inertia_sum <= real_t{0.0})
        return false;
    const real_t effective_carrier_inertia = real_t{4.0} / inverse_inertia_sum;
    if (!std::isfinite(effective_carrier_inertia) || effective_carrier_inertia <= real_t{0.0})
        return false;

    left = left_body;
    right = right_body;
    left_inertia = left_value;
    right_inertia = right_value;
    carrier_inertia = effective_carrier_inertia;
    solver.set_snapshot(DifferentialSolver::Snapshot(**data));
    configured = true;
    return true;
}

void AxleDifferential::reset() {
    solver.set_snapshot(DifferentialSolver::Snapshot());
    left = nullptr;
    right = nullptr;
    left_inertia = real_t{0.0};
    right_inertia = real_t{0.0};
    carrier_inertia = real_t{0.0};
    transmitted_torque = real_t{0.0};
    configured = false;
}

real_t AxleDifferential::get_carrier_velocity() const {
    if (!configured)
        return real_t{0.0};
    const real_t velocity = real_t{0.5} *
            (left->get_angular_velocity() + right->get_angular_velocity());
    return std::isfinite(velocity) ? velocity : real_t{0.0};
}

real_t AxleDifferential::get_predicted_carrier_velocity(real_t dt) const {
    const real_t current = get_carrier_velocity();
    if (!configured || !std::isfinite(dt) || dt <= real_t{0.0})
        return current;

    const real_t left_velocity = left->predict_angular_velocity(dt);
    const real_t right_velocity = right->predict_angular_velocity(dt);
    if (!std::isfinite(left_velocity) || !std::isfinite(right_velocity))
        return current;
    const real_t predicted = real_t{0.5} * (left_velocity + right_velocity);
    return std::isfinite(predicted) ? predicted : current;
}

void AxleDifferential::add_carrier_torque(real_t total_torque) {
    if (!configured || !std::isfinite(total_torque))
        return;
    const real_t per_side = total_torque * real_t{0.5};
    if (!std::isfinite(per_side))
        return;
    left->add_torque(per_side);
    right->add_torque(per_side);
    transmitted_torque += total_torque;
    if (!std::isfinite(transmitted_torque))
        transmitted_torque = real_t{0.0};
}

void AxleDifferential::add_carrier_impulse(real_t total_impulse, real_t dt) {
    if (!configured || !std::isfinite(total_impulse) ||
            !std::isfinite(dt) || dt <= real_t{0.0})
        return;
    const real_t per_side_torque = total_impulse * real_t{0.5} / dt;
    if (!std::isfinite(per_side_torque))
        return;
    left->add_torque(per_side_torque);
    right->add_torque(per_side_torque);
}

real_t AxleDifferential::solve_relative(real_t dt) {
    const real_t pending_transmission = transmitted_torque;
    transmitted_torque = real_t{0.0};
    if (!configured || !std::isfinite(dt) || dt <= real_t{0.0})
        return real_t{0.0};

    const real_t left_velocity = left->predict_angular_velocity(dt);
    const real_t right_velocity = right->predict_angular_velocity(dt);
    if (!std::isfinite(left_velocity) || !std::isfinite(right_velocity))
        return real_t{0.0};

    const DifferentialSolver::Input input{
        left_inertia,
        right_inertia,
        left_velocity,
        right_velocity,
        pending_transmission,
        get_predicted_carrier_velocity(dt),
        dt};
    const real_t impulse = solver.solve(input);
    if (!std::isfinite(impulse))
        return real_t{0.0};
    const real_t per_side_torque = impulse / dt;
    if (!std::isfinite(per_side_torque))
        return real_t{0.0};

    left->add_torque(per_side_torque);
    right->add_torque(-per_side_torque);
    return impulse;
}

} // namespace godot
