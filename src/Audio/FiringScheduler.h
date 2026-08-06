#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Audio/Waveguide.h"

namespace vehicle_audio {

constexpr std::size_t MAX_FIRING_PHASES = MAX_WAVEGUIDE_SOURCES;
constexpr float FOUR_STROKE_CYCLE_DEGREES = 720.0f;
constexpr float FIRING_PHASE_TOLERANCE = 1.0e-5f;

struct FiringPhase {
	float crank_degrees = 0.0f;
	SourceId source = INVALID_SOURCE;
	float amplitude = 1.0f;
};

struct FiringEvent {
	SourceId source = INVALID_SOURCE;
	float amplitude = 0.0f;
	float crank_degrees = 0.0f;
};

/// Fixed authored phase table for a four-stroke engine.  A phase event is
/// routed to its authored acoustic source ID; this class does not synthesize
/// or allocate audio data.
class FourStrokeFiringScheduler {
	std::array<FiringPhase, MAX_FIRING_PHASES> phases{};
	std::size_t phase_count = 0;
	float previous_phase = 0.0f;
	bool started = false;

	static float normalize_phase(float crank_degrees);

public:
	FourStrokeFiringScheduler() = default;

	/// Install the conventional 1-3-4-2, four-cylinder, 720-degree cycle.
	void configure_default_four_stroke();
	static FourStrokeFiringScheduler default_four_stroke();

	void clear();
	std::size_t get_phase_count() const { return phase_count; }
	bool set_phase(std::size_t index, float crank_degrees, SourceId source,
			float amplitude = 1.0f);
	bool append_phase(float crank_degrees, SourceId source,
			float amplitude = 1.0f);
	const FiringPhase &get_phase(std::size_t index) const { return phases[index]; }

	void reset(float crank_degrees = 0.0f);
	/// Stateless crossing query. `previous` and `current` are unwrapped within
	/// one forward cycle; wrap-around from near 720 to near zero is supported.
	std::size_t collect_events(float previous_crank_degrees,
			float current_crank_degrees, FiringEvent *events,
			std::size_t event_capacity) const;
	/// Stateful convenience wrapper. The first call establishes phase and emits
	/// no event; subsequent calls emit all authored phases crossed since it.
	std::size_t advance(float current_crank_degrees, FiringEvent *events,
			std::size_t event_capacity);
	/// Advance and inject events directly into a waveguide network.
	std::size_t advance_and_inject(float current_crank_degrees,
			WaveguideNetwork &network, float amplitude_scale = 1.0f);
};

} // namespace vehicle_audio
