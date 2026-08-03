#include "VehicleRunningGear.h"

#include <algorithm>

namespace godot {

void VehicleRunningGear::setup(const std::vector<Axle *> &setup_axles,
                               real_t mass,
                               const Ref<SuspensionData> &suspension,
                               const Ref<VehicleAerodynamicsData> &aero) {
    axles.clear();
    axles.reserve(setup_axles.size());
    for (Axle *axle : setup_axles)
        axles.push_back(axle);

    const real_t mass_per_axle = mass / static_cast<real_t>(axles.size());
    for (Axle *axle : axles)
        axle->compute_suspension_parameters(mass_per_axle, suspension);

    aerodynamics.load_parameters(aero);
    compute_axle_dimensions();
}

void VehicleRunningGear::update_suspension(RigidBody3D *vehicle,
                                           const Vector3 &body_origin,
                                           const Vector3 &com_global,
                                           const Vector3 &linear_velocity,
                                           const Vector3 &angular_velocity) {
    for (Axle *axle : axles) {
        axle->update_physics(com_global, linear_velocity, angular_velocity);

        const auto &wheels = axle->get_wheels();
        if (wheels.size() >= 2 && wheels[0]->is_on_ground() && wheels[1]->is_on_ground()) {
            const real_t force_0 = wheels[0]->get_suspension_rebound_force();
            const real_t force_1 = wheels[1]->get_suspension_rebound_force();

            // An anti-roll bar transfers load across the axle; it must neither
            // pull against the ground nor create net vertical force when a
            // wheel unloads.
            const real_t arb_force = std::clamp(
                axle->get_antiroll_bar_force(), -force_1, force_0);
            wheels[0]->set_normal_force(force_0 - arb_force);
            wheels[1]->set_normal_force(force_1 + arb_force);
        }

        for (Wheel *wheel : wheels) {
            if (!wheel->is_on_ground())
                continue;
            const Vector3 offset = wheel->collision_point - body_origin;
            vehicle->apply_force(wheel->collision_normal * wheel->get_suspension_rebound_force(), offset);
        }
    }
}

void VehicleRunningGear::apply_aerodynamics(RigidBody3D *vehicle,
                                            const Basis &body_basis,
                                            const Vector3 &linear_velocity,
                                            const Vector3 &angular_velocity,
                                            real_t vehicle_mass,
                                            const Vector3 &body_origin) {
    const AerodynamicsState state = aerodynamics.get_state(
        body_basis, linear_velocity, angular_velocity, vehicle_mass, axles);
    const AerodynamicForces forces = aerodynamics.compute(state);

    if (forces.drag.length_squared() > real_t{1e-8})
        vehicle->apply_central_force(forces.drag);
    if (forces.yaw_control_torque.length_squared() > real_t{1e-8})
        vehicle->apply_torque(forces.yaw_control_torque);
    apply_downforce(vehicle, forces.downforce, body_origin);
}

void VehicleRunningGear::apply_downforce(RigidBody3D *vehicle,
                                         real_t total_downforce,
                                         const Vector3 &body_origin) {
    if (total_downforce <= real_t{0.01})
        return;

    for (Axle *axle : axles) {
        const real_t axle_downforce = total_downforce * axle->get_downforce_ratio();
        if (axle_downforce < real_t{0.01})
            continue;

        const auto &wheels = axle->get_wheels();
        size_t grounded_wheels = 0;
        for (Wheel *wheel : wheels) {
            if (wheel->is_on_ground())
                ++grounded_wheels;
        }
        if (grounded_wheels == 0)
            continue;

        const real_t per_wheel = axle_downforce / static_cast<real_t>(grounded_wheels);
        for (Wheel *wheel : wheels) {
            if (!wheel->is_on_ground())
                continue;
            const Vector3 offset = wheel->collision_point - body_origin;
            vehicle->apply_force(wheel->collision_normal * -per_wheel, offset);
        }
    }
}

void VehicleRunningGear::solve_steering_and_tires(const Vector3 &com_global,
                                                  const Vector3 &linear_velocity,
                                                  const Vector3 &angular_velocity,
                                                  real_t dt,
                                                  real_t steer_input,
                                                  real_t wheel_brake,
                                                  real_t speed_kph,
                                                  bool abs_enabled) {
    for (Axle *axle : axles) {
        if (axle->get_steerable()) {
            axle->solve_steering(steer_input, dt, speed_kph);
            axle->set_wheels_rotation();
        }
        axle->solve_tire(com_global, linear_velocity, angular_velocity,
                         dt, wheel_brake, abs_enabled);
    }
}

void VehicleRunningGear::integrate_wheels(real_t dt) {
    for (Axle *axle : axles)
        axle->integrate(dt);
}

void VehicleRunningGear::apply_tire_forces(RigidBody3D *vehicle,
                                           const Vector3 &body_origin,
                                           int substeps) {
    const real_t inv_substeps = real_t{1.0} / static_cast<real_t>(substeps);
    for (Axle *axle : axles) {
        for (Wheel *wheel : axle->get_wheels()) {
            const Vector3 average_force = wheel->tire_force * inv_substeps;
            const Vector3 offset = wheel->collision_point - body_origin;
            vehicle->apply_force(average_force, offset);
            wheel->cached_tire_force = average_force;
            wheel->tire_force = Vector3(0, 0, 0);
        }
    }
}

void VehicleRunningGear::compute_axle_dimensions() {
    // Compute trackwidth for each axle (distance between its two wheels).
    for (Axle *axle : axles) {
        const auto &wheels = axle->get_wheels();
        if (wheels.size() >= 2) {
            const real_t trackwidth = wheels[0]->get_global_position().distance_to(
                wheels[1]->get_global_position());
            axle->set_trackwidth(trackwidth);
        }
    }

    // Compute wheelbase (maximum distance between any two axles).
    if (axles.size() < 2)
        return;

    real_t wheelbase = 0.0;
    for (size_t i = 0; i < axles.size(); ++i) {
        for (size_t j = i + 1; j < axles.size(); ++j) {
            const real_t distance = axles[i]->get_global_position().distance_to(
                axles[j]->get_global_position());
            wheelbase = std::max(wheelbase, distance);
        }
    }
    for (Axle *axle : axles)
        axle->set_wheelbase(wheelbase);
}

} // namespace godot
