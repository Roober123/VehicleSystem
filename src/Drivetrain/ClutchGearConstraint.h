#pragma once
#include "RotationalBody.h"
#include <cmath>
#include <vector>

namespace godot {

class ClutchGearConstraint {
	RotationalBody* engine = nullptr;
	RotationalBody* output = nullptr;
	bool aggregate_output_state_valid = false;
	real_t aggregate_output_inertia = 0.0;
	real_t aggregate_output_angular_velocity = 0.0;

 public:
	real_t clutch_engagement = 0.0;   // 0 = disengaged, 1 = fully locked
	real_t clutch_max_torque = 400.0; // Nm

	std::vector<real_t> gear_ratios = { 3.5, 2.1, 1.4, 1.0, 0.75 };
	real_t reverse_ratio = -3.0;        // Reverse gear ratio (negative for backward motion)
	int current_gear = 0;               // 0 = neutral, >0 = forward gears, <0 = reverse
	real_t final_drive = 3.73;

	void set_engine(RotationalBody* e)  { engine = e; }
	void set_output(RotationalBody* o)  { output = o; aggregate_output_state_valid = false; }
	void set_aggregate_output_state(real_t inertia, real_t angular_velocity) {
		aggregate_output_state_valid = std::isfinite(inertia) && inertia > real_t{0.0} &&
				std::isfinite(angular_velocity);
		aggregate_output_inertia = inertia;
		aggregate_output_angular_velocity = angular_velocity;
	}
	void clear_aggregate_output_state() { aggregate_output_state_valid = false; }
	RotationalBody* get_engine() const  { return engine; }
	RotationalBody* get_output() const  { return output; }

	real_t get_effective_ratio() const;
	void solve(real_t dt, real_t accumulated_engine_torque, real_t reflected_load_torque);
};

}
