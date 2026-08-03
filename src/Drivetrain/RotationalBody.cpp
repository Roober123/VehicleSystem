#include "RotationalBody.h"

namespace godot {

void RotationalBody::integrate(real_t dt) {
	angular_velocity += get_effective_torque() / inertia * dt;
	torque = 0.0;
}

real_t RotationalBody::predict_angular_velocity(real_t dt) const {
	return angular_velocity + get_effective_torque() / inertia * dt;
}

void RotationalBody::add_torque(real_t value) {
	torque += value;
}

void RotationalBody::set_inertia(real_t value) {
	inertia = value;
}

void RotationalBody::set_drag(real_t value) {
	drag = value;
}

void RotationalBody::set_angular_velocity(real_t value) {
	angular_velocity = value;
}

}
