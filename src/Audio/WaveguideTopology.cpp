#include "Audio/WaveguideTopology.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace vehicle_audio {
namespace {

bool finite(float value) {
	return std::isfinite(value);
}

bool same_name(const char *left, const char *right) {
	return left != nullptr && right != nullptr && std::strcmp(left, right) == 0;
}

} // namespace

bool validate_waveguide_topology(const WaveguideTopology &topology,
		float sample_rate, float propagation_speed,
		std::array<DerivedGuide, MAX_WAVEGUIDES> *derived) {
	const std::size_t node_count = topology.get_node_count();
	const std::size_t guide_count = topology.get_guide_count();
	const std::size_t source_count = topology.get_source_count();
	const std::size_t tap_count = topology.get_tap_count();
	if (node_count == 0 || node_count > MAX_WAVEGUIDE_NODES || guide_count == 0 ||
			guide_count > MAX_WAVEGUIDES || source_count > MAX_WAVEGUIDE_SOURCES ||
			tap_count > MAX_WAVEGUIDE_TAPS || !finite(sample_rate) ||
			!finite(propagation_speed) || sample_rate < 0.0f ||
			(sample_rate > 0.0f && propagation_speed <= 0.0f))
		return false;
	std::size_t delay_total = 0;
	std::array<std::uint8_t, MAX_WAVEGUIDE_NODES> degree{};
	for (std::size_t i = 0; i < node_count; ++i) {
		const NodeDefinition &node = topology.get_node(static_cast<NodeId>(i));
		if (node.name == nullptr || node.name[0] == '\0' || !finite(node.reflection) ||
				!finite(node.nonlinearity) || node.nonlinearity < 0.0f ||
				node.nonlinearity > MAX_JUNCTION_NONLINEARITY ||
				(node.type == NodeType::ReflectionBoundary &&
						(node.reflection < -1.0f || node.reflection > 1.0f)))
			return false;
		for (std::size_t j = 0; j < i; ++j)
			if (same_name(node.name,
					topology.get_node(static_cast<NodeId>(j)).name))
				return false;
	}
	for (std::size_t i = 0; i < guide_count; ++i) {
		const GuideDefinition &guide = topology.get_guide(static_cast<GuideId>(i));
		if (guide.name == nullptr || guide.name[0] == '\0' || guide.from >= node_count ||
				guide.to >= node_count || guide.from == guide.to ||
				!finite(guide.length_meters) || guide.length_meters <= 0.0f ||
				!finite(guide.area) || guide.area <= 0.0f ||
				!finite(guide.loss_per_meter) || guide.loss_per_meter < 0.0f ||
				!finite(guide.high_frequency_loss_per_meter) ||
				guide.high_frequency_loss_per_meter < 0.0f)
			return false;
		if (sample_rate > 0.0f) {
			const float raw_delay = guide.length_meters / propagation_speed * sample_rate;
			if (!finite(raw_delay) || raw_delay > static_cast<float>(MAX_DELAY_SAMPLES / 2))
				return false;
			const std::uint32_t delay = static_cast<std::uint32_t>(
					std::floor(raw_delay + 0.5f) < 1.0f ? 1.0f :
							std::floor(raw_delay + 0.5f));
			if (delay > MAX_DELAY_SAMPLES / 2 ||
					delay_total > MAX_DELAY_SAMPLES - 2 * delay)
				return false;
			if (derived != nullptr) {
				(*derived)[i].delay_samples = delay;
				const float attenuation = guide.loss_per_meter * guide.length_meters;
				const float high_frequency_attenuation =
						guide.high_frequency_loss_per_meter * guide.length_meters;
				if (!finite(attenuation) || !finite(high_frequency_attenuation))
					return false;
				(*derived)[i].traversal_loss = std::exp(-attenuation);
				(*derived)[i].high_frequency_alpha =
						std::exp(-high_frequency_attenuation);
			}
			delay_total += 2 * delay;
		}
		for (std::size_t j = 0; j < i; ++j)
			if (same_name(guide.name,
					topology.get_guide(static_cast<GuideId>(j)).name))
				return false;
		degree[guide.from]++;
		degree[guide.to]++;
	}
	for (std::size_t i = 0; i < node_count; ++i) {
		const NodeDefinition &node = topology.get_node(static_cast<NodeId>(i));
		if (node.type == NodeType::ReflectionBoundary && degree[i] != 1)
			return false;
	}
	for (std::size_t i = 0; i < source_count; ++i) {
		const SourceDefinition &source =
				topology.get_source(static_cast<SourceId>(i));
		if (source.name == nullptr || source.name[0] == '\0' || source.node >= node_count ||
				degree[source.node] == 0 || !finite(source.gain) ||
				!finite(source.pulse_width_ms) || source.pulse_width_ms <= 0.0f)
			return false;
		if (sample_rate > 0.0f) {
			const float pulse_samples = source.pulse_width_ms * sample_rate / 1000.0f;
			if (!finite(pulse_samples) || pulse_samples <= 0.0f ||
					pulse_samples >= static_cast<float>(std::numeric_limits<std::uint32_t>::max() / 8u))
				return false;
		}
		for (std::size_t j = 0; j < i; ++j)
			if (same_name(source.name,
					topology.get_source(static_cast<SourceId>(j)).name))
				return false;
	}
	for (std::size_t i = 0; i < tap_count; ++i) {
		const TapDefinition &tap = topology.get_tap(static_cast<TapId>(i));
		if (tap.name == nullptr || tap.name[0] == '\0' || tap.node >= node_count ||
				degree[tap.node] == 0 || !finite(tap.gain) ||
				(tap.mode == TapMode::BOUNDARY_RADIATION &&
						(topology.get_node(tap.node).type != NodeType::ReflectionBoundary ||
							degree[tap.node] != 1)))
			return false;
		for (std::size_t j = 0; j < i; ++j)
			if (same_name(tap.name,
				topology.get_tap(static_cast<TapId>(j)).name))
				return false;
	}

	// Multiple disconnected acoustic graphs are valid (for example, separate
	// intake and exhaust systems). Endpoint, geometry, and capacity checks
	// above are intentionally local; no global connected-component policy is
	// imposed here.
	return true;
}

bool WaveguideTopologyBuilder::valid_name(const char *name) const {
	return name != nullptr && name[0] != '\0';
}

void WaveguideTopologyBuilder::clear() {
	topology = WaveguideTopology{};
}

NodeId WaveguideTopologyBuilder::resolve_node(const char *name) const {
	if (!valid_name(name))
		return INVALID_NODE;
	for (std::size_t i = 0; i < topology.node_count; ++i)
		if (same_name(name, topology.nodes[i].name))
			return static_cast<NodeId>(i);
	return INVALID_NODE;
}

SourceId WaveguideTopologyBuilder::resolve_source(const char *name) const {
	if (!valid_name(name))
		return INVALID_SOURCE;
	for (std::size_t i = 0; i < topology.source_count; ++i)
		if (same_name(name, topology.sources[i].name))
			return static_cast<SourceId>(i);
	return INVALID_SOURCE;
}

TapId WaveguideTopologyBuilder::resolve_tap(const char *name) const {
	if (!valid_name(name))
		return INVALID_TAP;
	for (std::size_t i = 0; i < topology.tap_count; ++i)
		if (same_name(name, topology.taps[i].name))
			return static_cast<TapId>(i);
	return INVALID_TAP;
}


NodeId WaveguideTopologyBuilder::add_junction(const char *name,
		float nonlinearity) {
	if (!valid_name(name) || !finite(nonlinearity) || nonlinearity < 0.0f ||
			nonlinearity > MAX_JUNCTION_NONLINEARITY ||
		topology.node_count == MAX_WAVEGUIDE_NODES ||
			resolve_node(name) != INVALID_NODE)
		return INVALID_NODE;
	const NodeId id = static_cast<NodeId>(topology.node_count++);
	topology.nodes[id] = NodeDefinition{NodeType::Junction, 0.0f, name,
			nonlinearity};
	return id;
}

NodeId WaveguideTopologyBuilder::add_reflection_boundary(const char *name,
		float reflection) {
	if (!valid_name(name) || !finite(reflection) || reflection < -1.0f ||
			reflection > 1.0f || topology.node_count == MAX_WAVEGUIDE_NODES ||
			resolve_node(name) != INVALID_NODE)
		return INVALID_NODE;
	const NodeId id = static_cast<NodeId>(topology.node_count++);
	topology.nodes[id] = NodeDefinition{NodeType::ReflectionBoundary, reflection,
		name, 0.0f};
	return id;
}

GuideId WaveguideTopologyBuilder::add_guide(const char *name, NodeId from,
		NodeId to, float length_meters, float area, float loss_per_meter,
		float high_frequency_loss_per_meter) {
	if (!valid_name(name) || topology.guide_count == MAX_WAVEGUIDES ||
			from >= topology.node_count || to >= topology.node_count || from == to ||
				!finite(length_meters) || length_meters <= 0.0f || !finite(area) ||
				area <= 0.0f || !finite(loss_per_meter) || loss_per_meter < 0.0f ||
				!finite(high_frequency_loss_per_meter) ||
				high_frequency_loss_per_meter < 0.0f)
		return INVALID_GUIDE;
	for (std::size_t i = 0; i < topology.guide_count; ++i)
		if (same_name(name, topology.guides[i].name))
			return INVALID_GUIDE;
	const GuideId id = static_cast<GuideId>(topology.guide_count++);
	topology.guides[id] = GuideDefinition{name, from, to, length_meters, area,
			loss_per_meter, high_frequency_loss_per_meter};
	return id;
}

GuideId WaveguideTopologyBuilder::add_guide(const char *name,
		const char *from_name, const char *to_name, float length_meters, float area,
		float loss_per_meter, float high_frequency_loss_per_meter) {
	return add_guide(name, resolve_node(from_name), resolve_node(to_name),
			length_meters, area, loss_per_meter, high_frequency_loss_per_meter);
}

SourceId WaveguideTopologyBuilder::add_source(const char *name, NodeId node,
		float gain, float pulse_width_ms) {
	if (!valid_name(name) || topology.source_count == MAX_WAVEGUIDE_SOURCES ||
			node >= topology.node_count || !finite(gain) ||
			!finite(pulse_width_ms) || pulse_width_ms <= 0.0f)
		return INVALID_SOURCE;
	for (std::size_t i = 0; i < topology.source_count; ++i)
		if (same_name(name, topology.sources[i].name))
			return INVALID_SOURCE;
	const SourceId id = static_cast<SourceId>(topology.source_count++);
	topology.sources[id] = SourceDefinition{name, node, gain, pulse_width_ms};
	return id;
}

SourceId WaveguideTopologyBuilder::add_source(const char *name,
		const char *node_name, float gain, float pulse_width_ms) {
	return add_source(name, resolve_node(node_name), gain, pulse_width_ms);
}

TapId WaveguideTopologyBuilder::add_tap(const char *name, NodeId node,
		float gain) {
	return add_tap(name, node, TapMode::NODE_PRESSURE, gain);
}

TapId WaveguideTopologyBuilder::add_tap(const char *name, NodeId node,
		TapMode mode, float gain) {
	if (!valid_name(name) || topology.tap_count == MAX_WAVEGUIDE_TAPS ||
			node >= topology.node_count || !finite(gain) ||
			(mode == TapMode::BOUNDARY_RADIATION &&
				(node >= topology.node_count ||
					topology.nodes[node].type != NodeType::ReflectionBoundary)))
		return INVALID_TAP;
	for (std::size_t i = 0; i < topology.tap_count; ++i)
		if (same_name(name, topology.taps[i].name))
			return INVALID_TAP;
	const TapId id = static_cast<TapId>(topology.tap_count++);
	topology.taps[id] = TapDefinition{name, node, gain, mode};
	return id;
}

TapId WaveguideTopologyBuilder::add_tap(const char *name, const char *node_name,
		float gain) {
	return add_tap(name, resolve_node(node_name), TapMode::NODE_PRESSURE, gain);
}

TapId WaveguideTopologyBuilder::add_tap(const char *name,
		const char *node_name, TapMode mode, float gain) {
	return add_tap(name, resolve_node(node_name), mode, gain);
}

TapId WaveguideTopologyBuilder::add_boundary_radiation_tap(const char *name,
		NodeId node, float gain) {
	return add_tap(name, node, TapMode::BOUNDARY_RADIATION, gain);
}

TapId WaveguideTopologyBuilder::add_boundary_radiation_tap(const char *name,
		const char *node_name, float gain) {
	return add_boundary_radiation_tap(name, resolve_node(node_name), gain);
}

GuideId WaveguideTopologyBuilder::find_guide(const char *name) const {
	if (!valid_name(name))
		return INVALID_GUIDE;
	for (std::size_t i = 0; i < topology.guide_count; ++i)
		if (same_name(name, topology.guides[i].name))
			return static_cast<GuideId>(i);
	return INVALID_GUIDE;
}

bool WaveguideTopologyBuilder::build(WaveguideTopology &out) const {
	if (!validate_waveguide_topology(topology))
		return false;
	out = topology;
	return true;
}

bool WaveguideTopologyBuilder::build(WaveguideNetwork &out, float sample_rate,
		float propagation_speed) const {
	// The runtime configure path is the canonical validation/derivation pass;
	// avoid validating the same description twice when building directly into
	// a network.
	return out.configure(topology, sample_rate, propagation_speed);
}

} // namespace vehicle_audio
