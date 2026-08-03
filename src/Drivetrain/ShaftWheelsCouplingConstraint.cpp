#include "ShaftWheelsCouplingConstraint.h"

#include <cmath>

#include "axle.h"

namespace godot {

void ShaftWheelsCouplingConstraint::load_bodies(
        RotationalBody *veh_driveshaft,
        const std::vector<Axle *> &veh_axles,
        const Ref<DifferentialData> &center_data) {
    driveshaft = veh_driveshaft;
    ports = {};
    port_count = 0;
    effective_carrier_inertia = real_t{0.0};
    shaft_inertia = driveshaft != nullptr ? driveshaft->get_inertia() : real_t{0.0};
    center_solver.set_snapshot(center_data.is_valid()
            ? DifferentialSolver::Snapshot(**center_data)
            : DifferentialSolver::Snapshot());

    if (!std::isfinite(shaft_inertia) || shaft_inertia <= real_t{0.0}) {
        shaft_inertia = real_t{0.0};
        return;
    }

    real_t share_sum = real_t{0.0};
    for (Axle *axle : veh_axles) {
        if (port_count == MAX_DRIVEN_AXLES)
            break;
        if (axle == nullptr || !std::isfinite(axle->get_drive_share()) ||
                axle->get_drive_share() <= real_t{0.0})
            continue;

        AxleDifferential &differential = axle->get_differential();
        const real_t inertia = differential.get_carrier_inertia();
        if (!differential.is_configured() || !std::isfinite(inertia) ||
                inertia <= real_t{0.0})
            continue;

        ports[port_count++] = CarrierPort{
            &differential, inertia, axle->get_drive_share()};
        share_sum += axle->get_drive_share();
    }

    if (port_count == 0 || !std::isfinite(share_sum) || share_sum <= real_t{0.0}) {
        ports = {};
        port_count = 0;
        return;
    }

    real_t inverse_effective_inertia = real_t{0.0};
    for (size_t i = 0; i < port_count; ++i) {
        ports[i].drive_share /= share_sum;
        const real_t share = ports[i].drive_share;
        inverse_effective_inertia += share * share / ports[i].inertia;
    }
    if (!std::isfinite(inverse_effective_inertia) ||
            inverse_effective_inertia <= real_t{0.0}) {
        ports = {};
        port_count = 0;
        return;
    }
    effective_carrier_inertia = real_t{1.0} / inverse_effective_inertia;
    if (!std::isfinite(effective_carrier_inertia) ||
            effective_carrier_inertia <= real_t{0.0}) {
        ports = {};
        port_count = 0;
        effective_carrier_inertia = real_t{0.0};
    }
}

bool ShaftWheelsCouplingConstraint::center_is_locked() const {
    return port_count == MAX_DRIVEN_AXLES &&
            center_solver.get_snapshot().mode == DifferentialData::LOCKED;
}

real_t ShaftWheelsCouplingConstraint::get_aggregate_inertia() const {
    if (port_count == 0)
        return shaft_inertia;
    if (!center_is_locked())
        return shaft_inertia + effective_carrier_inertia;

    real_t aggregate = shaft_inertia;
    for (size_t i = 0; i < port_count; ++i)
        aggregate += ports[i].inertia;
    return std::isfinite(aggregate) ? aggregate : shaft_inertia;
}

real_t ShaftWheelsCouplingConstraint::project_shaft_velocity(
        bool predicted, real_t dt) const {
    if (driveshaft == nullptr)
        return real_t{0.0};

    const real_t shaft_velocity = predicted
            ? driveshaft->predict_angular_velocity(dt)
            : driveshaft->get_angular_velocity();
    if (!std::isfinite(shaft_velocity) || port_count == 0)
        return std::isfinite(shaft_velocity) ? shaft_velocity : real_t{0.0};

    if (center_is_locked()) {
        real_t momentum = shaft_inertia * shaft_velocity;
        real_t inertia = shaft_inertia;
        for (size_t i = 0; i < port_count; ++i) {
            const real_t velocity = predicted
                    ? ports[i].differential->get_predicted_carrier_velocity(dt)
                    : ports[i].differential->get_carrier_velocity();
            if (!std::isfinite(velocity))
                return shaft_velocity;
            momentum += ports[i].inertia * velocity;
            inertia += ports[i].inertia;
        }
        if (!std::isfinite(momentum) || !std::isfinite(inertia) || inertia <= real_t{0.0})
            return shaft_velocity;
        const real_t common_velocity = momentum / inertia;
        return std::isfinite(common_velocity) ? common_velocity : shaft_velocity;
    }

    real_t weighted_carrier_velocity = real_t{0.0};
    real_t primary_k = real_t{1.0} / shaft_inertia;
    for (size_t i = 0; i < port_count; ++i) {
        const real_t velocity = predicted
                ? ports[i].differential->get_predicted_carrier_velocity(dt)
                : ports[i].differential->get_carrier_velocity();
        if (!std::isfinite(velocity))
            return shaft_velocity;
        const real_t share = ports[i].drive_share;
        weighted_carrier_velocity += share * velocity;
        primary_k += share * share / ports[i].inertia;
    }
    if (!std::isfinite(weighted_carrier_velocity) || !std::isfinite(primary_k) ||
            primary_k <= real_t{0.0})
        return shaft_velocity;
    const real_t primary_impulse =
            -(shaft_velocity - weighted_carrier_velocity) / primary_k;
    const real_t projected = shaft_velocity + primary_impulse / shaft_inertia;
    return std::isfinite(projected) ? projected : shaft_velocity;
}

real_t ShaftWheelsCouplingConstraint::get_aggregate_angular_velocity() const {
    return project_shaft_velocity(false, real_t{0.0});
}

real_t ShaftWheelsCouplingConstraint::get_predicted_aggregate_angular_velocity(
        real_t sub_dt) const {
    if (!std::isfinite(sub_dt) || sub_dt <= real_t{0.0})
        return get_aggregate_angular_velocity();
    return project_shaft_velocity(true, sub_dt);
}

void ShaftWheelsCouplingConstraint::solve(real_t sub_dt) {
    if (driveshaft == nullptr || port_count == 0 ||
            !std::isfinite(sub_dt) || sub_dt <= real_t{0.0})
        return;

    const real_t primary_k = real_t{1.0} / shaft_inertia +
            real_t{1.0} / effective_carrier_inertia;
    const real_t aggregate_inertia = shaft_inertia + effective_carrier_inertia;
    const real_t pending_shaft_torque = driveshaft->get_pending_torque();
    const real_t transfer_fraction = effective_carrier_inertia / aggregate_inertia;
    const real_t transmitted_torque = pending_shaft_torque * transfer_fraction;
    if (!std::isfinite(primary_k) || primary_k <= real_t{0.0} ||
            !std::isfinite(aggregate_inertia) || aggregate_inertia <= real_t{0.0} ||
            !std::isfinite(pending_shaft_torque) ||
            !std::isfinite(transfer_fraction) ||
            !std::isfinite(transmitted_torque))
        return;

    std::array<real_t, MAX_DRIVEN_AXLES> routed_torques{};
    for (size_t i = 0; i < port_count; ++i) {
        routed_torques[i] = transmitted_torque * ports[i].drive_share;
        if (!std::isfinite(routed_torques[i]))
            return;
    }

    // Torque routing is one complete, prevalidated mutation stage. Predictions
    // below therefore include the exact torques that wheel integration sees.
    driveshaft->add_torque(-transmitted_torque);
    for (size_t i = 0; i < port_count; ++i)
        ports[i].differential->add_carrier_torque(routed_torques[i]);

    const real_t shaft_free_velocity = driveshaft->predict_angular_velocity(sub_dt);
    std::array<real_t, MAX_DRIVEN_AXLES> carrier_free_velocities{};
    real_t weighted_carrier_velocity = real_t{0.0};
    if (!std::isfinite(shaft_free_velocity))
        return;
    for (size_t i = 0; i < port_count; ++i) {
        carrier_free_velocities[i] =
                ports[i].differential->get_predicted_carrier_velocity(sub_dt);
        if (!std::isfinite(carrier_free_velocities[i]))
            return;
        weighted_carrier_velocity +=
                ports[i].drive_share * carrier_free_velocities[i];
    }
    if (!std::isfinite(weighted_carrier_velocity))
        return;

    const real_t primary_residual = shaft_free_velocity - weighted_carrier_velocity;
    const real_t primary_impulse = -primary_residual / primary_k;
    if (!std::isfinite(primary_residual) || !std::isfinite(primary_impulse))
        return;

    real_t center_impulse = real_t{0.0};
    real_t center_primary_reaction = real_t{0.0};
    if (port_count == MAX_DRIVEN_AXLES) {
        const real_t s0 = ports[0].drive_share;
        const real_t s1 = ports[1].drive_share;
        const real_t c0 = ports[0].inertia;
        const real_t c1 = ports[1].inertia;
        const real_t primary_center_coupling = -s0 / c0 + s1 / c1;
        const real_t center_k = real_t{1.0} / c0 + real_t{1.0} / c1;
        const real_t conditioned_center_k = center_k -
                primary_center_coupling * primary_center_coupling / primary_k;
        const real_t post_primary_velocity0 = carrier_free_velocities[0] -
                s0 * primary_impulse / c0;
        const real_t post_primary_velocity1 = carrier_free_velocities[1] -
                s1 * primary_impulse / c1;
        const real_t center_residual =
                post_primary_velocity0 - post_primary_velocity1;
        if (!std::isfinite(primary_center_coupling) || !std::isfinite(center_k) ||
                !std::isfinite(conditioned_center_k) ||
                conditioned_center_k <= real_t{0.0} ||
                !std::isfinite(center_residual))
            return;

        const real_t synthetic_inertia = real_t{2.0} / conditioned_center_k;
        const real_t projected_shaft_velocity =
                shaft_free_velocity + primary_impulse / shaft_inertia;
        if (!std::isfinite(synthetic_inertia) || synthetic_inertia <= real_t{0.0} ||
                !std::isfinite(projected_shaft_velocity))
            return;
        center_impulse = center_solver.solve(DifferentialSolver::Input{
            synthetic_inertia,
            synthetic_inertia,
            center_residual * real_t{0.5},
            -center_residual * real_t{0.5},
            transmitted_torque,
            projected_shaft_velocity,
            sub_dt});
        center_primary_reaction =
                -primary_center_coupling / primary_k * center_impulse;
        if (!std::isfinite(center_impulse) ||
                !std::isfinite(center_primary_reaction))
            return;
    }

    const real_t shaft_impulse = primary_impulse + center_primary_reaction;
    std::array<real_t, MAX_DRIVEN_AXLES> port_impulses{};
    for (size_t i = 0; i < port_count; ++i) {
        port_impulses[i] = -ports[i].drive_share *
                (primary_impulse + center_primary_reaction);
        if (port_count == MAX_DRIVEN_AXLES)
            port_impulses[i] += i == 0 ? center_impulse : -center_impulse;
        if (!std::isfinite(port_impulses[i]))
            return;
    }
    if (!std::isfinite(shaft_impulse))
        return;

    // Primary and center corrections are staged above, so a failed finite
    // check cannot leave a partially applied constraint impulse.
    driveshaft->add_torque(shaft_impulse / sub_dt);
    for (size_t i = 0; i < port_count; ++i)
        ports[i].differential->add_carrier_impulse(port_impulses[i], sub_dt);

    for (size_t i = 0; i < port_count; ++i)
        ports[i].differential->solve_relative(sub_dt);
}

} // namespace godot
