#pragma once

#include <vector>

#include "godot_cpp/classes/rigid_body3d.hpp"
#include "godot_cpp/core/defs.hpp"
#include "godot_cpp/variant/basis.hpp"
#include "godot_cpp/variant/vector3.hpp"

#include "Resources/suspension_data.h"
#include "Resources/vehicle_aerodynamics_data.h"
#include "axle.h"
#include "TractionControl.h"
#include "VehicleAerodynamics.h"

namespace godot {

/// Value-owned running-gear composition and setup-cached axle topology.
///
/// This component owns all wheel/suspension/steering/tire phases and the
/// vehicle-level aerodynamic/downforce summaries.  The topology is copied
/// once during setup and is never discovered from the scene during physics.
class VehicleRunningGear {
    std::vector<Axle *> axles;
    VehicleAerodynamics aerodynamics;
    TractionControl tcs;

    void compute_axle_dimensions();
    void apply_downforce(RigidBody3D *vehicle, real_t total_downforce,
                         const Vector3 &body_origin);

public:
    void setup(const std::vector<Axle *> &setup_axles, real_t mass,
               const Ref<SuspensionData> &suspension,
               const Ref<VehicleAerodynamicsData> &aero);

    const std::vector<Axle *> &get_axles() const { return axles; }

    real_t apply_traction_control(real_t driver_throttle, real_t speed) {
        return tcs.apply(driver_throttle, speed, axles);
    }

    void update_suspension(RigidBody3D *vehicle,
                           const Vector3 &body_origin,
                           const Vector3 &com_global,
                           const Vector3 &linear_velocity,
                           const Vector3 &angular_velocity);

    void apply_aerodynamics(RigidBody3D *vehicle,
                            const Vector3 &linear_velocity,
                            const Vector3 &body_origin);

    void solve_steering_and_tires(const Vector3 &com_global,
                                  const Vector3 &linear_velocity,
                                  const Vector3 &angular_velocity,
                                  real_t dt,
                                  real_t steer_input,
                                  real_t wheel_brake,
                                  real_t speed_kph,
                                  bool abs_enabled);

    void integrate_wheels(real_t dt);

    void apply_tire_forces(RigidBody3D *vehicle,
                           const Vector3 &body_origin,
                           int substeps);
};

} // namespace godot
