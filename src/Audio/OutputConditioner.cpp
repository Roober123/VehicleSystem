#include "Audio/OutputConditioner.h"

#include <cmath>

namespace vehicle_audio {

void OutputConditioner::reset() {
	dc_previous_input = 0.0f;
	dc_previous_output = 0.0f;
	low_pass_previous = 0.0f;
}

float OutputConditioner::process(float sample) {
	const float dc_blocked = sample - dc_previous_input +
			DC_BLOCKER_COEFFICIENT * dc_previous_output;
	dc_previous_input = sample;
	dc_previous_output = dc_blocked;
	const float low_passed = (1.0f - LOW_PASS_COEFFICIENT) * dc_blocked +
			LOW_PASS_COEFFICIENT * low_pass_previous;
	low_pass_previous = low_passed;
	return OUTPUT_GAIN * std::tanh(DRIVE * low_passed);
}

} // namespace vehicle_audio
