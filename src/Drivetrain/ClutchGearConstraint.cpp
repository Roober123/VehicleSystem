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

void ClutchGearConstraint::solve(real_t dt) {
	if (engine == nullptr || output == nullptr)
		return;

	const real_t ratio = get_effective_ratio();

	if (ratio == 0.0)
		return;

	const real_t engine_omega = engine->get_angular_velocity();
	const real_t reflected_omega = output->get_angular_velocity() * ratio;
	const real_t slip = engine_omega - reflected_omega;

	real_t clutch_torque = clutch_engagement * clutch_max_torque;

	if (slip < 0.0) {
		clutch_torque = -clutch_torque;
	}

	const real_t engine_inertia = engine->get_inertia();
	const real_t output_inertia = output->get_inertia();

	// Clamp only to prevent slip from reversing sign in one timestep
	// (numerical stability). The driveshaft inertia now includes wheel
	// inertias set by ShaftWheelsCouplingConstraint::load_bodies, so
	// the gear-ratio amplification no longer causes wild overshoot.
	const real_t combined_inertia = (engine_inertia * output_inertia)
	                              / (output_inertia + engine_inertia * ratio * ratio);
	const real_t max_stopping_torque = std::abs(slip) * combined_inertia / std::max(dt, real_t{1e-6});

	if (std::abs(clutch_torque) > max_stopping_torque) {
		clutch_torque = (clutch_torque > 0.0 ? 1.0 : -1.0) * max_stopping_torque;
	}

	engine->add_torque(-clutch_torque);
	output->add_torque(clutch_torque * ratio);
}

} // namespace godot
