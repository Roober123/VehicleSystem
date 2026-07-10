#pragma once
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/basis.hpp"
#include "godot_cpp/classes/resource.hpp"
#include "Resources/vehicle_aerodynamics_data.h"
#include <vector>

namespace godot {

class Axle;

struct AerodynamicsState {
    Basis body_basis;
    Vector3 linear_velocity;
    Vector3 angular_velocity;
    real_t vehicle_mass = 0.0;
    real_t wheelbase = 0.0;
    real_t trackwidth = 0.0;
    real_t steer_angle = 0.0;
    int grounded_wheels = 0;
};

struct AerodynamicForces {
    Vector3 drag;
    Vector3 yaw_control_torque;
    real_t downforce = 0.0;
};

class VehicleAerodynamics {
    
    static constexpr real_t air_density = 1.225;
    static constexpr real_t frontal_area = 2.2;

    real_t drag_coefficient = 0.35;
    real_t downforce_coefficient = 0.15;
    // Retains the old serialized name, but now controls desired-yaw feedback.
    real_t yaw_damping_coefficient = 2.0;
    real_t yaw_control_min_speed = 3.0;
    real_t yaw_control_max_torque = 6000.0;
    real_t yaw_control_max_lateral_acceleration = 9.81;

    Vector3 _compute_drag(const Vector3 &linear_velocity, real_t dynamic_pressure) const;
    Vector3 _compute_yaw_control(const AerodynamicsState &state) const;

public:
    AerodynamicsState get_state(const Basis &body_basis,
                                const Vector3 &linear_velocity,
                                const Vector3 &angular_velocity,
                                real_t vehicle_mass,
                                const std::vector<Axle*> &axles) const;
    AerodynamicForces compute(const AerodynamicsState &state) const;

    void load_parameters(const Ref<VehicleAerodynamicsData>& a);
    void set_drag_coefficient(real_t v) { drag_coefficient = v; }
    real_t get_drag_coefficient() const { return drag_coefficient; }

    void set_downforce_coefficient(real_t v) { downforce_coefficient = v; }
    real_t get_downforce_coefficient() const { return downforce_coefficient; }

    void set_yaw_damping_coefficient(real_t v) { yaw_damping_coefficient = v; }
    real_t get_yaw_damping_coefficient() const { return yaw_damping_coefficient; }
};

} // namespace godot
