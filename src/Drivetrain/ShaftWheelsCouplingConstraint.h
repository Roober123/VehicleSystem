#pragma once

#include <cmath>
#include <vector>

#include "RotationalBody.h"
#include "wheel.h"
#include "axle.h"

namespace godot {

class ShaftWheelsCouplingConstraint {
    struct DrivenWheelState {
        Wheel *wheel = nullptr;
        real_t inertia = 0.0;
    };

    RotationalBody *driveshaft = nullptr;
    // These containers are setup-time caches. The solve path only iterates
    // their already-reserved entries and performs no topology discovery.
    std::vector<Axle *> axles;
    std::vector<DrivenWheelState> driven_wheels;

    // The driveshaft retains its physical inertia; wheel inertia is never
    // folded into that independently integrated body.
    real_t total_wheel_inertia = 0.0;
    real_t drive_ratio_sum = 0.0;
    real_t shaft_inertia = 0.0;

    // Route a pending shaft/input torque to the wheel aggregate while
    // preserving the total output torque. The returned value is the torque
    // added to the wheel aggregate (and removed from the shaft).
    real_t route_pending_drive_torque();

public:
    void load_bodies(RotationalBody *veh_driveshaft,
                     const std::vector<Axle *> &veh_axles);

    /// Predict free velocities from all pending body torques, then apply a
    /// simultaneous rigid common-velocity impulse projection. The impulse is
    /// equal and opposite across the shaft and every driven wheel, conserving
    /// shaft-plus-wheel angular momentum and avoiding target crossing.
    real_t solve(real_t sub_dt);

    real_t get_aggregate_inertia() const {
        return shaft_inertia + total_wheel_inertia;
    }
    real_t get_aggregate_angular_velocity() const;
    /// Returns the inertia-weighted aggregate angular velocity after one
    /// unprojected substep using every currently pending shaft/wheel net
    /// torque, including each body's explicit drag at its current velocity.
    /// This is a read-only prediction for the clutch; solve() consumes the
    /// same pending torques exactly once when it applies the projection.
    real_t get_predicted_aggregate_angular_velocity(real_t sub_dt) const;
};

} // namespace godot
