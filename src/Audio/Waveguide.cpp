#include "Audio/Waveguide.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vehicle_audio {
namespace {

bool finite(float value) {
	return std::isfinite(value);
}

float pulse_normalization(float attack_width, float decay_width) {
	// The continuous two-exponential difference has a closed-form peak.  The
	// resulting scale is compiled once per source, so the render loop only
	// advances two bounded one-pole states.
	if (!(attack_width > 0.0f) || !(decay_width > attack_width))
		return 1.0f;
	const float peak_time = std::log(decay_width / attack_width) /
			(1.0f / attack_width - 1.0f / decay_width);
	const float peak = std::exp(-peak_time / decay_width) -
			std::exp(-peak_time / attack_width);
	return finite(peak) && peak > std::numeric_limits<float>::epsilon()
			? 1.0f / peak
			: 1.0f;
}

float junction_pressure(float linear_pressure, float nonlinearity) {
	if (!(nonlinearity > 0.0f) || !finite(linear_pressure))
		return linear_pressure;
	const float denominator = 1.0f + nonlinearity * std::abs(linear_pressure);
	if (finite(denominator) && denominator > 0.0f) {
		const float compressed = linear_pressure / denominator;
		if (finite(compressed))
			return compressed;
	}
	// Keep pathological finite inputs bounded even when the product above
	// overflows.  Normal authored values never take this fallback path.
	return std::copysign(1.0f / nonlinearity, linear_pressure);
}

} // namespace

bool WaveguideNetwork::configure(const WaveguideTopology &topology,
		float sample_rate, float propagation_speed) {
	std::array<DerivedGuide, MAX_GUIDES> derived{};
	if (!validate_waveguide_topology(topology, sample_rate, propagation_speed,
			&derived))
		return false;
	const std::size_t input_nodes = topology.get_node_count();
	const std::size_t input_guides = topology.get_guide_count();
	const std::size_t input_sources = topology.get_source_count();
	const std::size_t input_taps = topology.get_tap_count();
	// Commit only after every setup check succeeds, preserving the previous
	// network when a replacement topology is malformed.
	node_count = input_nodes;
	guide_count = input_guides;
	source_count = input_sources;
	tap_count = input_taps;
	used_delay_samples = 0;
	for (std::size_t i = 0; i < guide_count; ++i)
		used_delay_samples += 2 * derived[i].delay_samples;
	for (std::size_t i = 0; i < node_count; ++i)
		nodes[i] = topology.get_node(static_cast<NodeId>(i));
	for (std::size_t i = 0; i < guide_count; ++i) {
		guides[i].definition = topology.get_guide(static_cast<GuideId>(i));
		guides[i].offset = 0;
		for (std::size_t prior = 0; prior < i; ++prior)
			guides[i].offset += 2 * derived[prior].delay_samples;
		guides[i].cursor = 0;
		guides[i].delay_samples = derived[i].delay_samples;
		guides[i].traversal_loss = derived[i].traversal_loss;
		guides[i].high_frequency_alpha = derived[i].high_frequency_alpha;
		guides[i].filtered_from = 0.0f;
		guides[i].filtered_to = 0.0f;
	}
	for (std::size_t i = 0; i < source_count; ++i) {
		const SourceDefinition &definition =
				topology.get_source(static_cast<SourceId>(i));
		sources[i].definition = definition;
		const float width_samples = definition.pulse_width_ms * sample_rate / 1000.0f;
		const float attack_width = std::max(1.0e-4f, width_samples * 0.15f);
		const float decay_width = std::max(attack_width + 1.0e-4f, width_samples);
		sources[i].width_samples = width_samples;
		sources[i].attack_coefficient = std::exp(-1.0f / attack_width);
		sources[i].decay_coefficient = std::exp(-1.0f / decay_width);
		sources[i].normalization = pulse_normalization(attack_width, decay_width);
		sources[i].attack_ratio = 0.15f;
		const float silent_after = std::ceil(width_samples * 8.0f);
		sources[i].silent_after_samples = finite(silent_after) &&
				silent_after < static_cast<float>(std::numeric_limits<std::uint32_t>::max())
				? static_cast<std::uint32_t>(std::max(1.0f, silent_after))
				: std::numeric_limits<std::uint32_t>::max();
	}
	for (std::size_t i = 0; i < tap_count; ++i)
		taps[i] = topology.get_tap(static_cast<TapId>(i));

	endpoint_count.fill(0);
	for (std::size_t i = 0; i < guide_count; ++i) {
		const GuideDefinition &guide = guides[i].definition;
		const NodeId from = guide.from;
		const NodeId to = guide.to;
		endpoints[from][endpoint_count[from]++] =
				Endpoint{static_cast<GuideId>(i), true};
		endpoints[to][endpoint_count[to]++] =
				Endpoint{static_cast<GuideId>(i), false};
	}
	delay_samples.fill(0.0f);
	incoming_from.fill(0.0f);
	incoming_to.fill(0.0f);
	outgoing_from.fill(0.0f);
	outgoing_to.fill(0.0f);
	node_pressure.fill(0.0f);
	source_pressure.fill(0.0f);
	source_pending.fill(0.0f);
	source_pending_width_scale.fill(1.0f);
	source_pending_attack_ratio.fill(0.15f);
	source_attack_state.fill(0.0f);
	source_decay_state.fill(0.0f);
	source_event_attack_coefficient.fill(0.0f);
	source_event_decay_coefficient.fill(0.0f);
	source_event_normalization.fill(1.0f);
	source_event_silent_after_samples.fill(0);
	source_age_samples.fill(0);
	source_active.fill(false);
	node_incident.fill(0.0f);
	tap_values.fill(0.0f);
	configured = true;
	return true;
}

bool WaveguideNetwork::inject(SourceId source, float pressure) {
	if (!configured || source >= source_count || !finite(pressure))
		return false;
	return queue_source_event(source, pressure * sources[source].definition.gain,
			1.0f, sources[source].attack_ratio);
}

bool WaveguideNetwork::inject_event(SourceId source, float pressure, float rpm,
		float throttle, float strength_variation) {
	if (!configured || source >= source_count || !finite(pressure) ||
			!finite(rpm) || !finite(throttle) || !finite(strength_variation))
		return false;
	// RPM and throttle shape the event's valve-flow envelope, while remaining
	// bounded and preserving the authored source width as the zero-load
	// baseline.  This deliberately does not alter scheduler phase timing.
	const float rpm_load = std::clamp(rpm / 8000.0f, 0.0f, 1.0f);
	const float throttle_load = std::clamp(throttle, 0.0f, 1.0f);
	const float load = std::clamp(0.55f * rpm_load + 0.45f * throttle_load,
			0.0f, 1.0f);
	const float width_scale = std::clamp(1.0f - 0.35f * load, 0.65f, 1.0f);
	const float attack_ratio = std::clamp(0.15f - 0.04f * load, 0.10f, 0.15f);
	const float variation = std::clamp(strength_variation, 0.98f, 1.02f);
	const float scaled_pressure = pressure * variation * sources[source].definition.gain;
	return finite(scaled_pressure) && queue_source_event(source, scaled_pressure,
			width_scale, attack_ratio);
}

bool WaveguideNetwork::queue_source_event(SourceId source, float pressure,
		float width_scale, float attack_ratio) {
	if (!configured || source >= source_count || !finite(pressure) ||
			!finite(width_scale) || !finite(attack_ratio) || width_scale <= 0.0f ||
			attack_ratio <= 0.0f || attack_ratio >= 1.0f)
		return false;
	const float pending = source_pending[source] + pressure;
	if (!finite(pending))
		return false;
	// At most one bounded pending event-shape record is retained per source.
	// Multiple same-sample events still sum pressure deterministically; the
	// first shape remains the event-time state for that sample.
	if (source_pending[source] == 0.0f) {
		source_pending_width_scale[source] = width_scale;
		source_pending_attack_ratio[source] = attack_ratio;
	}
	source_pending[source] = pending;
	return true;
}

void WaveguideNetwork::clear_source_injections() {
	source_pending.fill(0.0f);
	source_pending_width_scale.fill(1.0f);
	source_pending_attack_ratio.fill(0.15f);
}

float WaveguideNetwork::read_from(GuideId guide_id) {
	const RuntimeGuide &runtime = guides[guide_id];
	const std::uint32_t length = runtime.delay_samples;
	const float delayed = delay_samples[runtime.offset + length + runtime.cursor];
	RuntimeGuide &state = guides[guide_id];
	state.filtered_from = state.high_frequency_alpha * delayed +
			(1.0f - state.high_frequency_alpha) * state.filtered_from;
	return state.filtered_from;
}

float WaveguideNetwork::read_to(GuideId guide_id) {
	const RuntimeGuide &runtime = guides[guide_id];
	const float delayed = delay_samples[runtime.offset + runtime.cursor];
	RuntimeGuide &state = guides[guide_id];
	state.filtered_to = state.high_frequency_alpha * delayed +
			(1.0f - state.high_frequency_alpha) * state.filtered_to;
	return state.filtered_to;
}

void WaveguideNetwork::write_sample(GuideId guide_id, float from, float to) {
	RuntimeGuide &runtime = guides[guide_id];
	delay_samples[runtime.offset + runtime.cursor] = from * runtime.traversal_loss;
	delay_samples[runtime.offset + runtime.delay_samples + runtime.cursor] =
			to * runtime.traversal_loss;
	++runtime.cursor;
	if (runtime.cursor == runtime.delay_samples)
		runtime.cursor = 0;
}

float WaveguideNetwork::process_sample() {
	// Convert pending source events into fixed two-exponential pulse samples.
	// All source/runtime bounds were checked during configure; this loop only
	// advances precompiled state and performs no topology validation.
	source_pressure.fill(0.0f);
	for (std::size_t i = 0; i < source_count; ++i) {
		RuntimeSource &source = sources[i];
		if (source_pending[i] != 0.0f) {
			if (!source_active[i]) {
				source_attack_state[i] = source_pending[i];
				source_decay_state[i] = source_pending[i];
				source_age_samples[i] = 0;
				const float width_samples = std::max(1.0e-4f,
						source.width_samples * source_pending_width_scale[i]);
				const float attack_width = std::max(1.0e-4f,
						width_samples * source_pending_attack_ratio[i]);
				const float decay_width = std::max(attack_width + 1.0e-4f,
						width_samples);
				source_event_attack_coefficient[i] = std::exp(-1.0f / attack_width);
				source_event_decay_coefficient[i] = std::exp(-1.0f / decay_width);
				source_event_normalization[i] = pulse_normalization(attack_width,
						decay_width);
				const float silent_after = std::ceil(width_samples * 8.0f);
				source_event_silent_after_samples[i] = finite(silent_after) &&
						silent_after < static_cast<float>(std::numeric_limits<std::uint32_t>::max())
						? static_cast<std::uint32_t>(std::max(1.0f, silent_after))
						: std::numeric_limits<std::uint32_t>::max();
				source_active[i] = true;
			} else {
				// Events are normally separated by many widths; summing into the
				// active state keeps overlapping authored events deterministic.
				source_attack_state[i] += source_pending[i];
				source_decay_state[i] += source_pending[i];
			}
			source_pending[i] = 0.0f;
		}
		if (!source_active[i])
			continue;
		source_attack_state[i] *= source_event_attack_coefficient[i];
		source_decay_state[i] *= source_event_decay_coefficient[i];
		const float pulse = source_event_normalization[i] *
				(source_decay_state[i] - source_attack_state[i]);
		if (finite(pulse))
			source_pressure[source.definition.node] += pulse;
		if (++source_age_samples[i] >= source_event_silent_after_samples[i]) {
			source_active[i] = false;
			source_attack_state[i] = 0.0f;
			source_decay_state[i] = 0.0f;
		}
	}
	for (std::size_t i = 0; i < guide_count; ++i) {
		incoming_from[i] = read_from(static_cast<GuideId>(i));
		incoming_to[i] = read_to(static_cast<GuideId>(i));
	}
	for (std::size_t n = 0; n < node_count; ++n) {
		float weighted = 0.0f;
		float area_sum = 0.0f;
		for (std::size_t e = 0; e < endpoint_count[n]; ++e) {
			const Endpoint endpoint = endpoints[n][e];
			const GuideDefinition &guide = guides[endpoint.guide].definition;
			const float incoming = endpoint.at_from ? incoming_from[endpoint.guide]
					: incoming_to[endpoint.guide];
			weighted += guide.area * incoming;
			area_sum += guide.area;
		}
		const float incident = (area_sum > 0.0f) ? (weighted / area_sum) : 0.0f;
		node_incident[n] = incident;
		float pressure = source_pressure[n];
		const NodeDefinition &node = nodes[n];
		if (node.type == NodeType::Junction) {
			const float linear = pressure +
					((area_sum > 0.0f) ? (2.0f * weighted / area_sum) : 0.0f);
			pressure = junction_pressure(linear, node.nonlinearity);
		} else {
			// A boundary tap observes total pressure: incident plus reflected
			// wave, with any authored source pulse added once.
			pressure += (1.0f + node.reflection) * incident;
		}
		node_pressure[n] = pressure;
		for (std::size_t e = 0; e < endpoint_count[n]; ++e) {
			const Endpoint endpoint = endpoints[n][e];
			const float incoming = endpoint.at_from ? incoming_from[endpoint.guide]
					: incoming_to[endpoint.guide];
			float outgoing = pressure - incoming;
			if (endpoint.at_from)
				outgoing_from[endpoint.guide] = outgoing;
			else
				outgoing_to[endpoint.guide] = outgoing;
		}
	}
	for (std::size_t i = 0; i < guide_count; ++i)
		write_sample(static_cast<GuideId>(i), outgoing_from[i], outgoing_to[i]);
	for (std::size_t i = 0; i < tap_count; ++i)
		tap_values[i] = taps[i].mode == TapMode::BOUNDARY_RADIATION
				? (1.0f - nodes[taps[i].node].reflection) *
						node_incident[taps[i].node] * taps[i].gain
				: node_pressure[taps[i].node] * taps[i].gain;
	const float result = tap_count > 0 ? tap_values[0] : 0.0f;
	source_pressure.fill(0.0f);
	return result;
}

bool WaveguideNetwork::render(float *output, std::size_t sample_count, TapId tap) {
	if (!configured || (sample_count > 0 && output == nullptr) || tap >= tap_count)
		return false;
	for (std::size_t i = 0; i < sample_count; ++i) {
		process_sample();
		output[i] = tap_values[tap];
	}
	return true;
}

bool WaveguideNetwork::render_all(float *output, std::size_t sample_count) {
	if (!configured || (sample_count > 0 && output == nullptr))
		return false;
	for (std::size_t i = 0; i < sample_count; ++i) {
		process_sample();
		for (std::size_t tap = 0; tap < tap_count; ++tap)
			output[i * tap_count + tap] = tap_values[tap];
	}
	return true;
}

float WaveguideNetwork::get_tap_value(TapId tap) const {
	return tap < tap_count ? tap_values[tap] : 0.0f;
}

float WaveguideNetwork::get_node_pressure(NodeId node) const {
	return node < node_count ? node_pressure[node] : 0.0f;
}

std::uint32_t WaveguideNetwork::get_guide_delay_samples(GuideId guide) const {
	return guide < guide_count ? guides[guide].delay_samples : 0;
}

float WaveguideNetwork::get_guide_traversal_loss(GuideId guide) const {
	return guide < guide_count ? guides[guide].traversal_loss : 0.0f;
}

float WaveguideNetwork::get_guide_high_frequency_alpha(GuideId guide) const {
	return guide < guide_count ? guides[guide].high_frequency_alpha : 0.0f;
}

float WaveguideNetwork::get_source_pulse_width_ms(SourceId source) const {
	return source < source_count ? sources[source].definition.pulse_width_ms : 0.0f;
}

} // namespace vehicle_audio
