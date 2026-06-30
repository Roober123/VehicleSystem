#include "RotationalBody.h"

namespace godot {

void RotationalBody::integrate(real_t dt) {
	real_t drag_torque = -drag * angular_velocity;
	angular_velocity += (torque + drag_torque) / inertia * dt;
	angle += angular_velocity * dt;
	torque = 0.0;
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