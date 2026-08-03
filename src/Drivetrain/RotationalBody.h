
#pragma once
#include <godot_cpp/core/defs.hpp>

namespace godot {

class RotationalBody {
protected:
	real_t angle = 0.0;
	real_t angular_velocity = 0.0;

	real_t inertia = 1.0;
	real_t torque = 0.0;
	real_t drag = 0.0;

public:
	void integrate(real_t dt);
	void add_torque(real_t value);
	void clear_torque() { torque = 0.0; }
	void set_inertia(real_t value);
	void set_drag(real_t value);
	void set_angular_velocity(real_t value);

	real_t get_angular_velocity() const { return angular_velocity; }
	real_t get_angle() const { return angle; }
	real_t get_inertia() const { return inertia; }
	real_t get_torque() const { return torque; }
	real_t get_drag() const { return drag; }
	/// Net torque used by integrate(), including explicit drag at the current
	/// angular velocity. Primary drivetrain routing intentionally uses
	/// get_torque() only; drag belongs to prediction/integration, not routing.
	real_t get_effective_torque() const { return torque - drag * angular_velocity; }
	/// Predict the exact explicit-Euler velocity that integrate(dt) will produce
	/// before clearing the pending torque accumulator.
	real_t predict_angular_velocity(real_t dt) const;
	real_t get_accumulated_torque() const { return torque; }
};

}
