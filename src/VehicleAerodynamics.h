#pragma once
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/classes/resource.hpp"
#include "Resources/vehicle_aerodynamics_data.h"

namespace godot {

struct AerodynamicForces {
    Vector3 drag;
    real_t downforce = 0.0;
};

class VehicleAerodynamics {
    static constexpr real_t air_density = 1.225;
    static constexpr real_t frontal_area = 2.2;

    real_t drag_coefficient = 0.35;
    real_t downforce_coefficient = 0.15;

    Vector3 _compute_drag(const Vector3 &linear_velocity, real_t dynamic_pressure) const;

public:
    AerodynamicForces compute(const Vector3 &linear_velocity) const;

    void load_parameters(const Ref<VehicleAerodynamicsData>& a);
};

} // namespace godot
