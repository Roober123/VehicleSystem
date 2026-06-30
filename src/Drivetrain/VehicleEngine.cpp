#include "VehicleEngine.h"
#include <algorithm>

namespace godot {

real_t VehicleEngine::get_rpm_normalized() {
	if (torque_curve.is_null()) return 0.0;
	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;
	real_t t = (rpm - idle_rpm) / (redline_rpm - idle_rpm);
	t = std::clamp(t, real_t{0.0}, real_t{1.0});
	return t;
}

real_t VehicleEngine::get_torque() {
	if (torque_curve.is_null()) return 0.0;
	real_t tq_point = get_rpm_normalized();
	return torque_curve->sample_baked(tq_point) * throttle * max_torque * (1.0 + turbo->get_boost());
}

real_t VehicleEngine::get_available_torque() {
	if (torque_curve.is_null()) return 0.0;
	real_t tq_point = get_rpm_normalized();
	real_t tq = torque_curve->sample_baked(tq_point) * max_torque;
	if (turbo != nullptr) tq = tq * (1.0 + turbo->get_boost());
	return tq;
}

void VehicleEngine::accumulate_torque(real_t dt) {


	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;

	real_t effective_throttle = throttle;
	if (rev_limit_cut)
		effective_throttle = real_t{0.0};
	
	
	if (turbo != nullptr)
		turbo->update(dt, rpm, effective_throttle);
	RotationalBody::add_torque(get_available_torque() * effective_throttle);

	// Internal friction drag
	if (rpm > idle_rpm)
		RotationalBody::add_torque(-drag * angular_velocity);

	// Engine braking
	if (effective_throttle < real_t{0.01} && rpm > idle_rpm) {
		real_t brake_torque = engine_braking * inertia * (rpm / redline_rpm) * 150;
		RotationalBody::add_torque(-brake_torque);
	}

	// Idle controller
	if (rpm < idle_rpm) {
		real_t idle_torque = max_torque * real_t{0.1} * (real_t{1.0} - rpm / idle_rpm);
		RotationalBody::add_torque(idle_torque);
	}
}

void VehicleEngine::integrate(real_t dt) {
	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;
	rev_limit_timer -= dt;

	if (rpm > redline_rpm) {
		rev_limit_cut = true;
		rev_limit_timer = 0.2;
	}
	else if (rpm < redline_rpm && rev_limit_timer <= 0)
		rev_limit_cut = false;
	
	

	angular_velocity += torque / inertia * dt;
	angle += angular_velocity * dt;
	torque = 0.0;

	constexpr real_t hard_limit_margin = real_t{1.05};
	if (rpm > redline_rpm * hard_limit_margin)
		angular_velocity = redline_rpm * hard_limit_margin / ang_to_rpm;
}

real_t VehicleEngine::get_current_horsepower() {
	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	constexpr real_t HP_FACTOR = 5252.0 / 0.737562;

	real_t rpm = angular_velocity * ang_to_rpm;
	return get_current_torque() * rpm / HP_FACTOR;
}

real_t VehicleEngine::get_turbo_boost() {
	if (turbo == nullptr) return 0.0;
	return turbo->get_boost();
}

real_t VehicleEngine::get_current_torque() {
	if (torque_curve.is_null()) return 0.0;
	real_t tq_point = get_rpm_normalized();
	real_t tq = torque_curve->sample_baked(tq_point) * throttle * max_torque;
	if (turbo != nullptr) tq *= real_t{1.0} + turbo->get_boost();
	return tq;
}

}