#include "VehicleEngine.h"
#include <algorithm>

namespace godot {

real_t VehicleEngine::get_rpm_normalized() {
	if (torque_curve.is_null()) return 0.0;
	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;
	real_t t = (rpm - idle_rpm) / (redline_rpm - idle_rpm);
	t = std::clamp<real_t>(t, real_t{0.0}, real_t{1.0});
	return t;
}

real_t VehicleEngine::get_torque() {
	if (torque_curve.is_null()) return 0.0;
	real_t tq_point = get_rpm_normalized();
	return torque_curve->sample_baked(tq_point) * throttle * max_torque;
}

real_t VehicleEngine::get_available_torque() {
	if (torque_curve.is_null()) return 0.0;
	real_t tq_point = get_rpm_normalized();
	return torque_curve->sample_baked(tq_point) * max_torque;
}

void VehicleEngine::accumulate_torque() {
	RotationalBody::add_torque(get_torque());

	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;
	if (rpm < idle_rpm) {
		real_t idle_torque = max_torque * 0.1 * (1.0 - rpm / idle_rpm);
		RotationalBody::add_torque(idle_torque);
	}
}

void VehicleEngine::integrate(real_t dt) {
	RotationalBody::integrate(dt);
	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;
	if (rpm > redline_rpm)
		angular_velocity = redline_rpm / ang_to_rpm;
}

} // namespace godot