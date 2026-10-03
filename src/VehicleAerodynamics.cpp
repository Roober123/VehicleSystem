#include "VehicleAerodynamics.h"

namespace godot {

AerodynamicForces VehicleAerodynamics::compute(const Vector3 &linear_velocity) const {
    const real_t speed_sq = linear_velocity.length_squared();
    const real_t dynamic_pressure = real_t{0.5} * air_density * speed_sq;

    AerodynamicForces forces;
    forces.drag = _compute_drag(linear_velocity, dynamic_pressure);
    forces.downforce = dynamic_pressure * downforce_coefficient * frontal_area;
    return forces;
}

Vector3 VehicleAerodynamics::_compute_drag(const Vector3 &linear_velocity, real_t dynamic_pressure) const {
    const real_t speed = linear_velocity.length();
    if (speed <= real_t{0.1})
        return Vector3();

    const real_t magnitude = dynamic_pressure * drag_coefficient * frontal_area;
    return -linear_velocity / speed * magnitude;
}

void VehicleAerodynamics::load_parameters(const Ref<VehicleAerodynamicsData>& a) {
    if (a.is_null()) {
        drag_coefficient = 0.0;
        downforce_coefficient = 0.0;
        return;
    }
    drag_coefficient = a->get_drag_coefficient();
    downforce_coefficient = a->get_downforce_coefficient();
}

} // namespace godot
