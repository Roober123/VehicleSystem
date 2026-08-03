#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "Drivetrain/RotationalBody.h"
#include "Drivetrain/axle_differential.h"
#include "Drivetrain/differential_solver.h"

namespace godot {

class Axle;

class ShaftWheelsCouplingConstraint {
    static constexpr size_t MAX_DRIVEN_AXLES = 2;

    struct CarrierPort {
        AxleDifferential *differential = nullptr;
        real_t inertia = 0.0;
        real_t drive_share = 0.0;
    };

    RotationalBody *driveshaft = nullptr;
    std::array<CarrierPort, MAX_DRIVEN_AXLES> ports{};
    size_t port_count = 0;
    DifferentialSolver center_solver;
    real_t effective_carrier_inertia = 0.0;
    real_t shaft_inertia = 0.0;

    bool center_is_locked() const;
    real_t project_shaft_velocity(bool predicted, real_t dt) const;

public:
    void load_bodies(RotationalBody *veh_driveshaft,
                     const std::vector<Axle *> &veh_axles,
                     const Ref<DifferentialData> &center_data);

    /// Route pending shaft torque, solve the primary shaft/carrier constraint,
    /// solve the optional center relative coordinate, then solve each axle's
    /// left/right relative coordinate. All topology is cached during setup.
    void solve(real_t sub_dt);

    real_t get_aggregate_inertia() const;
    real_t get_aggregate_angular_velocity() const;
    real_t get_predicted_aggregate_angular_velocity(real_t sub_dt) const;
};

} // namespace godot
