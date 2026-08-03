#include "ShaftWheelsCouplingConstraint.h"

namespace godot {

void ShaftWheelsCouplingConstraint::load_bodies(
        RotationalBody *veh_driveshaft,
        const std::vector<Axle *> &veh_axles) {
    driveshaft = veh_driveshaft;
    axles.clear();
    driven_wheels.clear();
    total_wheel_inertia = real_t{0.0};
    drive_ratio_sum = real_t{0.0};
    shaft_inertia = real_t{0.0};

    if (driveshaft == nullptr)
        return;

    axles.reserve(veh_axles.size());
    size_t wheel_capacity = 0;
    for (Axle *axle : veh_axles) {
        if (axle != nullptr)
            wheel_capacity += axle->get_wheels().size();
    }
    driven_wheels.reserve(wheel_capacity);

    // Discover the driven topology once during setup. Every wheel cached here
    // is the same physical body that receives normalized primary-drive torque
    // and the residual kinematic correction in solve().
    for (Axle *axle : veh_axles) {
        if (axle == nullptr || !std::isfinite(axle->drive_ratio) ||
                axle->drive_ratio <= real_t{0.0})
            continue;

        bool has_valid_wheel = false;
        for (Wheel *wheel : axle->get_wheels()) {
            if (wheel == nullptr)
                continue;
            const real_t inertia = wheel->body.get_inertia();
            if (!std::isfinite(inertia) || inertia <= real_t{0.0})
                continue;
            driven_wheels.push_back(DrivenWheelState{wheel, inertia});
            total_wheel_inertia += inertia;
            has_valid_wheel = true;
        }
        if (has_valid_wheel) {
            axles.push_back(axle);
            drive_ratio_sum += axle->drive_ratio;
        }
    }

    shaft_inertia = driveshaft->get_inertia();
    if (!std::isfinite(shaft_inertia) || shaft_inertia <= real_t{0.0})
        shaft_inertia = real_t{0.0};
    if (!std::isfinite(total_wheel_inertia) || total_wheel_inertia < real_t{0.0})
        total_wheel_inertia = real_t{0.0};
    if (!std::isfinite(drive_ratio_sum) || drive_ratio_sum < real_t{0.0})
        drive_ratio_sum = real_t{0.0};
}

real_t ShaftWheelsCouplingConstraint::get_aggregate_angular_velocity() const {
    if (driveshaft == nullptr || !std::isfinite(shaft_inertia) ||
            shaft_inertia <= real_t{0.0})
        return driveshaft != nullptr ? driveshaft->get_angular_velocity() : real_t{0.0};

    real_t angular_momentum =
            shaft_inertia * driveshaft->get_angular_velocity();
    for (const DrivenWheelState &state : driven_wheels) {
        if (state.wheel == nullptr || !std::isfinite(state.inertia) ||
                state.inertia <= real_t{0.0})
            continue;
        angular_momentum += state.inertia *
                state.wheel->body.get_angular_velocity();
    }

    const real_t aggregate_inertia = get_aggregate_inertia();
    if (!std::isfinite(angular_momentum) ||
            !std::isfinite(aggregate_inertia) || aggregate_inertia <= real_t{0.0})
        return driveshaft->get_angular_velocity();
    return angular_momentum / aggregate_inertia;
}

real_t ShaftWheelsCouplingConstraint::get_predicted_aggregate_angular_velocity(
        real_t sub_dt) const {
    const real_t current = get_aggregate_angular_velocity();
    if (driveshaft == nullptr || !std::isfinite(sub_dt) || sub_dt <= real_t{0.0} ||
            !std::isfinite(shaft_inertia) || shaft_inertia <= real_t{0.0} ||
            (!driven_wheels.empty() &&
             (!std::isfinite(total_wheel_inertia) ||
              total_wheel_inertia <= real_t{0.0})))
        return current;

    const real_t shaft_free_velocity =
            driveshaft->predict_angular_velocity(sub_dt);
    if (!std::isfinite(shaft_free_velocity))
        return current;
    if (driven_wheels.empty())
        return shaft_free_velocity;

    real_t free_momentum = shaft_inertia * shaft_free_velocity;
    for (const DrivenWheelState &state : driven_wheels) {
        if (state.wheel == nullptr || !std::isfinite(state.inertia) ||
                state.inertia <= real_t{0.0})
            return current;
        const real_t free_velocity =
                state.wheel->body.predict_angular_velocity(sub_dt);
        if (!std::isfinite(free_velocity))
            return current;
        free_momentum += state.inertia * free_velocity;
    }

    const real_t aggregate_inertia = get_aggregate_inertia();
    if (!std::isfinite(free_momentum) || !std::isfinite(aggregate_inertia) ||
            aggregate_inertia <= real_t{0.0})
        return current;
    const real_t predicted = free_momentum / aggregate_inertia;
    return std::isfinite(predicted) ? predicted : current;
}

real_t ShaftWheelsCouplingConstraint::route_pending_drive_torque() {
    if (driveshaft == nullptr || axles.empty() ||
            !std::isfinite(shaft_inertia) || shaft_inertia <= real_t{0.0} ||
            !std::isfinite(total_wheel_inertia) ||
            total_wheel_inertia <= real_t{0.0} ||
            !std::isfinite(drive_ratio_sum) || drive_ratio_sum <= real_t{0.0})
        return real_t{0.0};

    const real_t pending_shaft_torque = driveshaft->get_torque();
    const real_t aggregate_inertia = get_aggregate_inertia();
    if (!std::isfinite(pending_shaft_torque) ||
            !std::isfinite(aggregate_inertia) || aggregate_inertia <= real_t{0.0})
        return real_t{0.0};

    // Preserve the clutch/input torque on the aggregate while biasing the
    // nominal shaft/wheel shares by physical inertia. The axle share remains
    // a normalized routing contract and is independent from constraint
    // reaction impulses applied below.
    const real_t wheel_torque = pending_shaft_torque *
            total_wheel_inertia / aggregate_inertia;
    if (!std::isfinite(wheel_torque))
        return real_t{0.0};

    driveshaft->add_torque(-wheel_torque);
    for (Axle *axle : axles) {
        const real_t share = axle->drive_ratio / drive_ratio_sum;
        if (std::isfinite(share) && share > real_t{0.0})
            axle->add_torque(wheel_torque * share);
    }
    return wheel_torque;
}

real_t ShaftWheelsCouplingConstraint::solve(real_t sub_dt) {
    if (driveshaft == nullptr || axles.empty() || driven_wheels.empty() ||
            !std::isfinite(sub_dt) || sub_dt <= real_t{0.0} ||
            !std::isfinite(shaft_inertia) || shaft_inertia <= real_t{0.0} ||
            !std::isfinite(total_wheel_inertia) ||
            total_wheel_inertia <= real_t{0.0})
        return real_t{0.0};

    route_pending_drive_torque();

    const real_t aggregate_inertia = get_aggregate_inertia();
    if (!std::isfinite(aggregate_inertia) || aggregate_inertia <= real_t{0.0})
        return real_t{0.0};

    // First pass: predict all free velocities and their total momentum. The
    // RotationalBody prediction includes explicit drag exactly once, matching
    // the subsequent integrate() call. No body is integrated or mutated until
    // the common target is known.
    const real_t shaft_free_velocity =
            driveshaft->predict_angular_velocity(sub_dt);
    if (!std::isfinite(shaft_free_velocity))
        return real_t{0.0};
    real_t free_momentum = shaft_inertia * shaft_free_velocity;
    for (const DrivenWheelState &state : driven_wheels) {
        if (state.wheel == nullptr || !std::isfinite(state.inertia) ||
                state.inertia <= real_t{0.0})
            return real_t{0.0};
        const real_t free_velocity =
                state.wheel->body.predict_angular_velocity(sub_dt);
        if (!std::isfinite(free_velocity))
            return real_t{0.0};
        free_momentum += state.inertia * free_velocity;
    }

    if (!std::isfinite(free_momentum))
        return real_t{0.0};
    const real_t common_velocity = free_momentum / aggregate_inertia;
    if (!std::isfinite(common_velocity))
        return real_t{0.0};

    // Second pass: apply equal/opposite impulse corrections. The weighted
    // common velocity lies between the free body velocities, so this rigid
    // projection cannot overshoot or inject kinetic energy.
    const real_t shaft_impulse = shaft_inertia *
            (common_velocity - shaft_free_velocity);
    if (!std::isfinite(shaft_impulse))
        return real_t{0.0};
    driveshaft->add_torque(shaft_impulse / sub_dt);

    real_t wheel_impulse_sum = real_t{0.0};
    for (const DrivenWheelState &state : driven_wheels) {
        const real_t free_velocity =
                state.wheel->body.predict_angular_velocity(sub_dt);
        const real_t impulse = state.inertia *
                (common_velocity - free_velocity);
        if (!std::isfinite(impulse))
            continue;
        wheel_impulse_sum += impulse;
        state.wheel->body.add_torque(impulse / sub_dt);
    }

    // Round-off in the per-wheel sum can leave a tiny residual correction.
    // Close it on the shaft so the complete correction impulse remains exactly
    // equal/opposite at the aggregate level.
    const real_t residual_impulse = shaft_impulse + wheel_impulse_sum;
    if (std::isfinite(residual_impulse) && residual_impulse != real_t{0.0})
        driveshaft->add_torque(-residual_impulse / sub_dt);
    return (shaft_impulse - residual_impulse) / sub_dt;
}

} // namespace godot
