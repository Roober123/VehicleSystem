#include "ShaftWheelsCouplingConstraint.h"

namespace godot {

void ShaftWheelsCouplingConstraint::load_bodies(RotationalBody* veh_driveshaft, std::vector<Axle*> veh_axles) {
    axles.clear();
    driveshaft = veh_driveshaft;

    // Compute total wheel inertia from driven axles and collect them
    real_t I_wheels = 0.0;
    for (auto & ax : veh_axles) {
        if (ax->drive_ratio > 0.0) {
            axles.push_back(ax);
            for (auto* wh : ax->get_wheels()) {
                I_wheels += wh->body.get_inertia();
            }
        }
    }

    const real_t I_ds_original = driveshaft->get_inertia();
    driveshaft->set_inertia(I_ds_original + I_wheels);

    // precompute stiffness and damping
    const real_t I_ds = driveshaft->get_inertia();
    const real_t I_eff = (I_ds * I_wheels) / (I_ds + I_wheels);
    coupling_stiffness = omega_n * omega_n * I_eff;
    coupling_damping   = real_t{2.0} * zeta * omega_n * I_eff;
}

void ShaftWheelsCouplingConstraint::solve() {
    real_t total_torque = driveshaft->get_torque();
    for (auto & ax : axles) {
        real_t distributed_torque = ax->drive_ratio * total_torque;
        ax->add_torque(distributed_torque);
        driveshaft->add_torque(-distributed_torque);
    }

    
    const int axle_count = static_cast<int>(axles.size());
    if (axle_count == 0) return;

    for (auto & ax : axles) {
        const real_t axle_avg = ax->get_average_wheel_omega();
        const real_t angular_velocity_diff = axle_avg - driveshaft->get_angular_velocity();
        const real_t angular_displacement = ax->get_average_wheel_angle() - driveshaft->get_angle();
        if (std::abs(angular_velocity_diff) < real_t{1e-8} &&
            std::abs(angular_displacement) < real_t{1e-8}) continue;

        const real_t correction = (coupling_stiffness * angular_velocity_diff +
                                   coupling_damping * angular_velocity_diff) / axle_count;
        ax->add_torque(-correction);
        driveshaft->add_torque(correction);
    }
}

}
