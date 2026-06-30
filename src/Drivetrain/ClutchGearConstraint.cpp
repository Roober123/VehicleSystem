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

void ClutchGearConstraint::solve(real_t dt, real_t engine_torque, real_t reflected_load_torque) {
	if (engine == nullptr || output == nullptr)
		return;

	const real_t r = get_effective_ratio();
	if (r == 0.0)
		return; 
	const real_t angular_velocity_e = engine->get_angular_velocity();
	const real_t angular_velocity_o = output->get_angular_velocity();
	const real_t I_e = engine->get_inertia();
	const real_t I_o = output->get_inertia();
	const real_t angular_velocity_ref = angular_velocity_o * r;
	const real_t I_ref = I_o / (r * r);

	const real_t I_total   = I_e + I_ref;
	const real_t momentum  = I_e * angular_velocity_e + I_ref * angular_velocity_ref;
	const real_t torque_net= engine_torque - reflected_load_torque;
	const real_t angular_velocity_target  = (momentum + torque_net * dt) / I_total;


	real_t torque_c = engine_torque - I_e * (angular_velocity_target - angular_velocity_e) / dt;

	const real_t capacity = clutch_engagement * clutch_max_torque;
	torque_c = std::clamp(torque_c, -capacity, capacity);

	//const real_t slip   = angular_velocity_e - angular_velocity_ref;
	//const real_t I_eff  = (I_e * I_o) / (I_o + I_e * r * r);
	//const real_t max_stopping = std::abs(slip) * I_eff / std::max(dt, real_t{1e-6});
	//torque_c = std::clamp(torque_c, -max_stopping, max_stopping);

	engine->add_torque(-torque_c);
	output->add_torque(torque_c * r);
}

} 
