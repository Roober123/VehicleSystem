#include "SteeringRack.h"

namespace godot {

void SteeringRack::load(const Ref<SteeringRackData>& s) {
    inertia = s->get_inertia();
    damping = s->get_damping();
    friction_coefficient = s->get_friction_coefficient();
    max_angle = Math::deg_to_rad(s->get_max_angle());
    proportional_gain = s->get_proportional_gain();
    derivative_gain = s->get_derivative_gain();
    sat_gain = s->get_sat_gain();
}

void SteeringRack::solve(real_t steer_input, real_t sat_torque) {
    
}


}