#pragma once
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/classes/resource.hpp"
#include "Resources/vehicle_aerodynamics_data.h"

namespace godot {

class VehicleAerodynamics {
    
    static constexpr real_t air_density = 1.225;
    static constexpr real_t frontal_area = 2.2;

    real_t drag_coefficient = 0.35;
    real_t downforce_coefficient = 0.15;
    real_t yaw_damping_coefficient = 2.0;

    real_t drag_force = 0.0;
    real_t downforce = 0.0;
    real_t yaw_damping_factor = 0.0; // torque per rad/s of yaw

public:
    void compute(const Vector3 &linear_velocity);
    real_t get_yaw_torque(real_t yaw_rate, real_t vehicle_mass) const;

    void load_parameters(const Ref<VehicleAerodynamicsData>& a);
    real_t get_drag_force() const { return drag_force; }
    real_t get_downforce() const { return downforce; }
    real_t get_yaw_damping_factor() const { return yaw_damping_factor; }

    void set_drag_coefficient(real_t v) { drag_coefficient = v; }
    real_t get_drag_coefficient() const { return drag_coefficient; }

    void set_downforce_coefficient(real_t v) { downforce_coefficient = v; }
    real_t get_downforce_coefficient() const { return downforce_coefficient; }

    void set_yaw_damping_coefficient(real_t v) { yaw_damping_coefficient = v; }
    real_t get_yaw_damping_coefficient() const { return yaw_damping_coefficient; }
};

} // namespace godot
