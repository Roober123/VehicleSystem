#include "Audio/FiringScheduler.h"

#include <cmath>

namespace vehicle_audio {

float FourStrokeFiringScheduler::normalize_phase(float crank_degrees) {
	float phase = std::fmod(crank_degrees, FOUR_STROKE_CYCLE_DEGREES);
	if (phase < 0.0f)
		phase += FOUR_STROKE_CYCLE_DEGREES;
	return phase;
}

void FourStrokeFiringScheduler::configure_default_four_stroke() {
	clear();
	// Conventional four-cylinder firing order 1-3-4-2, represented by
	// zero-based source IDs 0-2-3-1 around the 720-degree cycle.
	append_phase(0.0f, static_cast<SourceId>(0));
	append_phase(180.0f, static_cast<SourceId>(2));
	append_phase(360.0f, static_cast<SourceId>(3));
	append_phase(540.0f, static_cast<SourceId>(1));
}

FourStrokeFiringScheduler FourStrokeFiringScheduler::default_four_stroke() {
	FourStrokeFiringScheduler result;
	result.configure_default_four_stroke();
	return result;
}

void FourStrokeFiringScheduler::clear() {
	phases.fill(FiringPhase{});
	phase_count = 0;
	previous_phase = 0.0f;
	started = false;
}

bool FourStrokeFiringScheduler::set_phase(std::size_t index,
		float crank_degrees, SourceId source, float amplitude) {
	if (index >= MAX_FIRING_PHASES || !std::isfinite(crank_degrees) ||
			source >= MAX_WAVEGUIDE_SOURCES || !std::isfinite(amplitude))
		return false;
	phases[index] = FiringPhase{normalize_phase(crank_degrees), source, amplitude};
	if (index >= phase_count)
		phase_count = index + 1;
	return true;
}

bool FourStrokeFiringScheduler::append_phase(float crank_degrees,
		SourceId source, float amplitude) {
	return set_phase(phase_count, crank_degrees, source, amplitude);
}

void FourStrokeFiringScheduler::reset(float crank_degrees) {
	previous_phase = normalize_phase(crank_degrees);
	started = true;
}

std::size_t FourStrokeFiringScheduler::collect_events(
		float previous_crank_degrees, float current_crank_degrees,
		FiringEvent *events, std::size_t event_capacity) const {
	if (!std::isfinite(previous_crank_degrees) ||
			!std::isfinite(current_crank_degrees) ||
			(event_capacity > 0 && events == nullptr))
		return 0;
	const float previous = normalize_phase(previous_crank_degrees);
	const float current = normalize_phase(current_crank_degrees);
	const bool wrapped = current < previous - FIRING_PHASE_TOLERANCE;
	std::size_t emitted = 0;
	for (std::size_t i = 0; i < phase_count; ++i) {
		const float phase = phases[i].crank_degrees;
		const bool crossed = wrapped
				? (phase > previous + FIRING_PHASE_TOLERANCE ||
						phase <= current + FIRING_PHASE_TOLERANCE)
				: (phase > previous + FIRING_PHASE_TOLERANCE &&
						phase <= current + FIRING_PHASE_TOLERANCE);
		if (!crossed)
			continue;
		if (emitted < event_capacity)
			events[emitted] = FiringEvent{phases[i].source, phases[i].amplitude,
					phase};
		++emitted;
	}
	return emitted;
}

std::size_t FourStrokeFiringScheduler::advance(float current_crank_degrees,
		FiringEvent *events, std::size_t event_capacity) {
	if (!std::isfinite(current_crank_degrees) ||
			(event_capacity > 0 && events == nullptr))
		return 0;
	const float current = normalize_phase(current_crank_degrees);
	if (!started) {
		previous_phase = current;
		started = true;
		return 0;
	}
	const std::size_t emitted = collect_events(previous_phase, current, events,
			event_capacity);
	previous_phase = current;
	return emitted < event_capacity ? emitted : event_capacity;
}

std::size_t FourStrokeFiringScheduler::advance_and_inject(
		float current_crank_degrees, WaveguideNetwork &network,
		float amplitude_scale) {
	if (!std::isfinite(amplitude_scale))
		return 0;
	std::array<FiringEvent, MAX_FIRING_PHASES> events{};
	const std::size_t event_count = advance(current_crank_degrees, events.data(),
			events.size());
	std::size_t injected = 0;
	for (std::size_t i = 0; i < event_count; ++i)
		if (network.inject(events[i].source, events[i].amplitude * amplitude_scale))
			++injected;
	return injected;
}

} // namespace vehicle_audio
