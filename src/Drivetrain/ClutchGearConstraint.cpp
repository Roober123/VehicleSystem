#include "ClutchGearConstraint.h"
#include <algorithm>
#include <cmath>

namespace godot {

real_t ClutchGearConstraint::get_effective_ratio() const {
	if (current_gear == 0)
		return 0.0;
	if (current_gear < 0)
		return reverse_ratio * final_drive;
	int index = current_gear - 1;
	if (index < 0 || index >= static_cast<int>(gear_ratios.size()))
		return final_drive;
	return gear_ratios[index] * final_drive;
}

void ClutchGearConstraint::solve(real_t dt, real_t accumulated_engine_torque, real_t reflected_load_torque) {
	if (engine == nullptr || output == nullptr ||
			!std::isfinite(dt) || dt <= real_t{0.0})
		return;

	const real_t r = get_effective_ratio();
	if (!std::isfinite(r) || std::abs(r) <= real_t{1e-8})
		return;

	const real_t I_e = engine->get_inertia();
	const real_t I_o = aggregate_output_state_valid
			? aggregate_output_inertia
			: output->get_inertia();
	if (!std::isfinite(I_e) || !std::isfinite(I_o) ||
			I_e <= real_t{0.0} || I_o <= real_t{0.0})
		return;

	if (!std::isfinite(accumulated_engine_torque) ||
			!std::isfinite(reflected_load_torque))
		return;

	const real_t angular_velocity_e = engine->get_angular_velocity();
	const real_t angular_velocity_o = aggregate_output_state_valid
			? aggregate_output_angular_velocity
			: output->get_angular_velocity();
	if (!std::isfinite(angular_velocity_e) || !std::isfinite(angular_velocity_o))
		return;

	// Reflect output inertia and resisting load into the engine-side
	// generalized coordinate.  The free relative motion includes both engine
	// drive torque and the reflected output load; this is the state the clutch
	// impulse is allowed to correct during this step.
	const real_t I_ref = I_o / (r * r);
	const real_t I_eff = (I_e * I_ref) / (I_e + I_ref);
	if (!std::isfinite(I_ref) || !std::isfinite(I_eff) ||
			I_ref <= real_t{0.0} || I_eff <= real_t{0.0})
		return;

	const real_t current_slip = angular_velocity_e - r * angular_velocity_o;
	const real_t free_slip = current_slip + dt *
			(accumulated_engine_torque / I_e + reflected_load_torque / I_ref);
	if (!std::isfinite(current_slip) || !std::isfinite(free_slip))
		return;

	// The impulse needed to stop free relative motion also carries the
	// synchronized holding torque (for example Te == reflected load at zero
	// slip).  Clamp that impulse directly, then convert it back to torque.
	real_t clutch_impulse = I_eff * free_slip;
	const real_t engagement = std::max(clutch_engagement, real_t{0.0});
	const real_t max_torque = std::max(clutch_max_torque, real_t{0.0});
	const real_t capacity = engagement * max_torque;
	const real_t max_impulse = capacity * dt;
	if (!std::isfinite(clutch_impulse) || !std::isfinite(max_impulse))
		return;
	clutch_impulse = std::clamp(clutch_impulse, -max_impulse, max_impulse);
	// Leave one representable unit of impulse on the predicted-slip side when
	// correcting nonzero slip.  Without this conservative step, float32
	// cancellation in the two body integrations can turn an exact zero into a
	// tiny sign reversal (especially for reverse ratios).  Synchronized holding
	// torque is left exact so equal engine/load torques remain transferable.
	if (std::abs(current_slip) > real_t{0.0} && clutch_impulse != real_t{0.0})
		clutch_impulse = std::nextafter(clutch_impulse, real_t{0.0});
	const real_t torque_c = clutch_impulse / dt;

	engine->add_torque(-torque_c);
	output->add_torque(torque_c * r);
}

} 
