#pragma once

namespace vehicle_audio {

/// Fixed output conditioning for the generated exterior-exhaust signal.
///
/// The coefficients are compiled for the VehicleAudio 48 kHz clock.  The
/// object owns only three scalar state values and process() performs no
/// allocation or topology validation, making it suitable for deterministic
/// native tests as well as the generator render loop.
class OutputConditioner {
public:
	static constexpr float DC_BLOCKER_COEFFICIENT = 0.997385430f; // 20 Hz
	static constexpr float LOW_PASS_COEFFICIENT = 0.427052633f; // 6.5 kHz
	static constexpr float OUTPUT_GAIN = 0.6f;
	static constexpr float DRIVE = 1.5f;

	OutputConditioner() = default;

	void reset();
	float process(float sample);

	float get_dc_previous_input() const { return dc_previous_input; }
	float get_dc_previous_output() const { return dc_previous_output; }
	float get_low_pass_previous() const { return low_pass_previous; }

private:
	float dc_previous_input = 0.0f;
	float dc_previous_output = 0.0f;
	float low_pass_previous = 0.0f;
};

} // namespace vehicle_audio
