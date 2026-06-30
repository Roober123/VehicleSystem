#include "VehicleAerodynamics.h"

namespace godot {

void VehicleAerodynamics::compute(const Vector3 &linear_velocity) {
    real_t speed_sq = linear_velocity.length_squared();

    // q = 0.5 * rho * v^2  (dynamic pressure)
    real_t q = real_t{0.5} * air_density * speed_sq;

    drag_force = q * drag_coefficient * frontal_area;
    downforce = q * downforce_coefficient * frontal_area;
    yaw_damping_factor = speed_sq * yaw_damping_coefficient;
}

real_t VehicleAerodynamics::get_yaw_torque(real_t yaw_rate, real_t vehicle_mass) const {
    return -vehicle_mass * yaw_rate * std::abs(yaw_rate) * yaw_damping_coefficient;
}

void VehicleAerodynamics::load_parameters(const Ref<VehicleAerodynamicsData>& a) {
    if (a.is_null())
        return;
    drag_coefficient = a->get_drag_coefficient();
    downforce_coefficient = a->get_downforce_coefficient();
    yaw_damping_coefficient = a->get_yaw_damping_coefficient();
}

} // namespace godot