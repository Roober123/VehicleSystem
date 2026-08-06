#include "VehicleEngine.h"
#include <algorithm>

namespace godot {

namespace {

constexpr real_t kRevLimiterHysteresisRpm = real_t{150.0};
constexpr real_t kEmergencyLimitRatio = real_t{1.10};

} // namespace

real_t VehicleEngine::get_rpm_normalized() const {
	if (torque_curve.is_null()) return 0.0;
	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;
	real_t t = (rpm - idle_rpm) / (redline_rpm - idle_rpm);
	t = std::clamp(t, real_t{0.0}, real_t{1.0});
	return t;
}

real_t VehicleEngine::get_torque() const {
	return effective_drive_torque;
}

void VehicleEngine::accumulate_torque(real_t dt) {


	constexpr real_t ang_to_rpm = 60.0 / (2.0 * Math_PI);
	real_t rpm = angular_velocity * ang_to_rpm;

	if (!rev_limit_cut && rpm >= redline_rpm)
		rev_limit_cut = true;
	else if (rev_limit_cut && rpm <= redline_rpm - kRevLimiterHysteresisRpm)
		rev_limit_cut = false;

	real_t effective_throttle = throttle;
	if (rev_limit_cut)
		effective_throttle = real_t{0.0};
	
	
	// Sample the engine curve once for this substep. The cached result is the
	// exact drive torque applied below, including rev cut, turbo boost, and
	// effective throttle, so telemetry does not resample the curve.
	const real_t curve_multiplier = torque_curve.is_null() ? real_t{0.0} : torque_curve->sample_baked(get_rpm_normalized());

	if (turbo != nullptr) {
		const real_t normalized_base_torque =
				std::clamp(curve_multiplier, real_t{0.0}, real_t{1.0});
		// A limiter cut stops combustion torque without masquerading as the
		// driver closing the throttle and venting boost.
		turbo->update(dt, rpm, throttle, normalized_base_torque);
	}

	const real_t air_charge_ratio = turbo != nullptr ? turbo->get_air_charge_ratio() : real_t{1.0};
	const real_t boosted_torque = curve_multiplier * max_torque * air_charge_ratio;
	effective_drive_torque = boosted_torque * effective_throttle;
	RotationalBody::add_torque(effective_drive_torque);

	// Engine braking
	if (effective_throttle < real_t{0.01} && rpm > idle_rpm) {
		real_t brake_torque = engine_braking * inertia * (rpm / redline_rpm) * 150;
		RotationalBody::add_torque(-brake_torque);
	}

	// At closed throttle, feed the configured idle speed's rotational drag
	// forward.  This cancels the inherited RotationalBody drag exactly at the
	// idle target; the proportional controller below still supplies recovery
	// torque whenever the engine falls under target speed.
	if (effective_throttle < real_t{0.01} && rpm <= idle_rpm) {
		constexpr real_t rpm_to_ang = 2.0 * Math_PI / 60.0;
		const real_t idle_feedforward = drag * idle_rpm * rpm_to_ang;
		RotationalBody::add_torque(idle_feedforward);
	}

	// Idle controller
	if (rpm < idle_rpm) {
		real_t idle_torque = max_torque * real_t{0.1} * (real_t{1.0} - rpm / idle_rpm);
		RotationalBody::add_torque(idle_torque);
	}
}

void VehicleEngine::integrate(real_t dt) {
	RotationalBody::integrate(dt);

	constexpr real_t rpm_to_ang = 2.0 * Math_PI / 60.0;
	const real_t emergency_limit = redline_rpm * kEmergencyLimitRatio * rpm_to_ang;
	if (angular_velocity > emergency_limit)
		angular_velocity = emergency_limit;
}

real_t VehicleEngine::get_turbo_boost() const {
	if (turbo == nullptr) return 0.0;
	return turbo->get_boost();
}

}
