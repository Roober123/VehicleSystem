#pragma once
#include "RotationalBody.h"
#include <vector>

namespace godot {

class ClutchGearConstraint {
	RotationalBody* engine = nullptr;
	RotationalBody* output = nullptr;

 public:
	real_t clutch_engagement = 0.0;   // 0 = disengaged, 1 = fully locked
	real_t clutch_max_torque = 400.0; // Nm

	std::vector<real_t> gear_ratios = { 3.5, 2.1, 1.4, 1.0, 0.75 };
	real_t reverse_ratio = -3.0;        // Reverse gear ratio (negative for backward motion)
	int current_gear = 0;               // 0 = neutral, >0 = forward gears, <0 = reverse
	real_t final_drive = 3.73;

	void set_engine(RotationalBody* e)  { engine = e; }
	void set_output(RotationalBody* o)  { output = o; }
	RotationalBody* get_engine() const  { return engine; }
	RotationalBody* get_output() const  { return output; }

	real_t get_effective_ratio() const;
	void solve(real_t dt, real_t engine_torque, real_t reflected_load_torque);
};

}