#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vehicle_audio {

// The audio graph is deliberately a small, fixed value object.  These limits
// are part of the runtime contract: setup rejects a graph which exceeds any
// one of them, while the render loop never needs to validate its topology.
constexpr std::size_t MAX_WAVEGUIDE_NODES = 32;
constexpr std::size_t MAX_WAVEGUIDES = 48;
constexpr std::size_t MAX_WAVEGUIDE_SOURCES = 16;
constexpr std::size_t MAX_WAVEGUIDE_TAPS = 4;
constexpr std::size_t MAX_DELAY_SAMPLES = 32768;
constexpr float DEFAULT_SAMPLE_RATE = 48000.0f;
constexpr float DEFAULT_PROPAGATION_SPEED = 343.0f;
constexpr float DEFAULT_PULSE_WIDTH_MS = 1.5f;

using NodeId = std::uint8_t;
using GuideId = std::uint8_t;
using SourceId = std::uint8_t;
using TapId = std::uint8_t;

constexpr NodeId INVALID_NODE = 0xffu;
constexpr GuideId INVALID_GUIDE = 0xffu;
constexpr SourceId INVALID_SOURCE = 0xffu;
constexpr TapId INVALID_TAP = 0xffu;

enum class NodeType : std::uint8_t {
	Junction,
	ReflectionBoundary,
};

enum class TapMode : std::uint8_t {
	NODE_PRESSURE,
	BOUNDARY_RADIATION,
};

struct NodeDefinition {
	NodeType type = NodeType::Junction;
	float reflection = 0.0f;
	const char *name = nullptr;
	/// Optional positive soft-compression strength for a junction.  A zero
	/// value is an exact linear identity; reflection boundaries ignore it.
	float nonlinearity = 0.0f;
};

struct GuideDefinition {
	const char *name = nullptr;
	NodeId from = INVALID_NODE;
	NodeId to = INVALID_NODE;
	/// Physical authoring values. Delay samples and traversal attenuation are
	/// derived by WaveguideNetwork::configure for the requested audio clock.
	float length_meters = 0.0f;
	float area = 0.0f;
	float loss_per_meter = 0.0f;
	float high_frequency_loss_per_meter = 0.0f;
};

struct SourceDefinition {
	const char *name = nullptr;
	NodeId node = INVALID_NODE;
	float gain = 1.0f;
	float pulse_width_ms = DEFAULT_PULSE_WIDTH_MS;
};

struct TapDefinition {
	const char *name = nullptr;
	NodeId node = INVALID_NODE;
	float gain = 1.0f;
	TapMode mode = TapMode::NODE_PRESSURE;
};

/// Immutable-after-build description consumed by WaveguideNetwork.
class WaveguideTopology {
	std::array<NodeDefinition, MAX_WAVEGUIDE_NODES> nodes{};
	std::array<GuideDefinition, MAX_WAVEGUIDES> guides{};
	std::array<SourceDefinition, MAX_WAVEGUIDE_SOURCES> sources{};
	std::array<TapDefinition, MAX_WAVEGUIDE_TAPS> taps{};
	std::size_t node_count = 0;
	std::size_t guide_count = 0;
	std::size_t source_count = 0;
	std::size_t tap_count = 0;

	friend class WaveguideTopologyBuilder;

public:
	WaveguideTopology() = default;

	std::size_t get_node_count() const { return node_count; }
	std::size_t get_guide_count() const { return guide_count; }
	std::size_t get_source_count() const { return source_count; }
	std::size_t get_tap_count() const { return tap_count; }
	const NodeDefinition &get_node(NodeId id) const { return nodes[id]; }
	const GuideDefinition &get_guide(GuideId id) const { return guides[id]; }
	const SourceDefinition &get_source(SourceId id) const { return sources[id]; }
	const TapDefinition &get_tap(TapId id) const { return taps[id]; }
};

/// Setup-derived guide state.  It is intentionally separate from authored
/// physical geometry so sample-rate changes never mutate the topology data.
struct DerivedGuide {
	std::uint32_t delay_samples = 0;
	float traversal_loss = 1.0f;
	float high_frequency_alpha = 1.0f;
};

/// Canonical setup validation shared by the builder and runtime.  When
/// `sample_rate` is zero only sample-independent topology facts are checked;
/// otherwise the output receives each guide's integer delay and traversal
/// attenuation exactly once.
bool validate_waveguide_topology(const WaveguideTopology &topology,
		float sample_rate = 0.0f,
		float propagation_speed = DEFAULT_PROPAGATION_SPEED,
		std::array<DerivedGuide, MAX_WAVEGUIDES> *derived = nullptr);

/// Fixed-memory bidirectional delay-line network.  Sources are pressure
/// injections at graph nodes; taps observe node pressure after junction
/// scattering.  No Godot type, heap allocation, or polymorphic node is used.
class WaveguideNetwork {
public:
	static constexpr std::size_t MAX_NODES = MAX_WAVEGUIDE_NODES;
	static constexpr std::size_t MAX_GUIDES = MAX_WAVEGUIDES;
	static constexpr std::size_t MAX_SOURCES = MAX_WAVEGUIDE_SOURCES;
	static constexpr std::size_t MAX_TAPS = MAX_WAVEGUIDE_TAPS;
	static constexpr std::size_t MAX_DELAY = MAX_DELAY_SAMPLES;

	WaveguideNetwork() = default;

	/// Copies a physical topology and allocates no storage. The setup pass is
	/// the only place geometry/capacity/endpoint checks and sample-derived
	/// delay calculations are performed.
	bool configure(const WaveguideTopology &topology,
			float sample_rate = DEFAULT_SAMPLE_RATE,
			float propagation_speed = DEFAULT_PROPAGATION_SPEED);
	bool is_configured() const { return configured; }

	/// Add one source pulse to the next rendered sample.  Multiple injections
	/// before a sample are summed.  Returns false for an invalid source id.
	bool inject(SourceId source, float pressure);
	/// Queue one firing event with a bounded RPM/load-shaped two-exponential
	/// envelope.  The authored source pulse width remains the baseline; RPM
	/// and throttle only derive this event's finite attack/decay state.
	bool inject_event(SourceId source, float pressure, float rpm,
			float throttle, float strength_variation = 1.0f);
	void clear_source_injections();

	/// Render one sample and return tap zero (or zero when no tap is authored).
	float process_sample();
	/// Render one tap into a caller-owned buffer.  The buffer is never allocated
	/// or resized by this method.
	bool render(float *output, std::size_t sample_count, TapId tap = 0);
	/// Interleaved render for all authored taps: frame i, tap j is
	/// output[i * tap_count + j].
	bool render_all(float *output, std::size_t sample_count);

	float get_tap_value(TapId tap) const;
	float get_node_pressure(NodeId node) const;
	std::uint32_t get_guide_delay_samples(GuideId guide) const;
	float get_guide_traversal_loss(GuideId guide) const;
	float get_guide_high_frequency_alpha(GuideId guide) const;
	float get_source_pulse_width_ms(SourceId source) const;
	std::size_t get_used_delay_samples() const { return used_delay_samples; }

private:
	struct RuntimeGuide {
		GuideDefinition definition{};
		std::size_t offset = 0;
		std::uint32_t cursor = 0;
		std::uint32_t delay_samples = 0;
		float traversal_loss = 1.0f;
		float high_frequency_alpha = 1.0f;
		float filtered_from = 0.0f;
		float filtered_to = 0.0f;
	};

	struct RuntimeSource {
		SourceDefinition definition{};
		float attack_coefficient = 0.0f;
		float decay_coefficient = 0.0f;
		float normalization = 1.0f;
		float width_samples = 1.0f;
		float attack_ratio = 0.15f;
		std::uint32_t silent_after_samples = 8;
	};

	struct Endpoint {
		GuideId guide = INVALID_GUIDE;
		bool at_from = false;
	};

	std::array<NodeDefinition, MAX_NODES> nodes{};
	std::array<RuntimeGuide, MAX_GUIDES> guides{};
	std::array<RuntimeSource, MAX_SOURCES> sources{};
	std::array<TapDefinition, MAX_TAPS> taps{};
	std::array<std::array<Endpoint, MAX_GUIDES>, MAX_NODES> endpoints{};
	std::array<std::uint8_t, MAX_NODES> endpoint_count{};
	std::array<float, MAX_DELAY> delay_samples{};
	std::array<float, MAX_GUIDES> incoming_from{};
	std::array<float, MAX_GUIDES> incoming_to{};
	std::array<float, MAX_GUIDES> outgoing_from{};
	std::array<float, MAX_GUIDES> outgoing_to{};
	std::array<float, MAX_NODES> node_pressure{};
	std::array<float, MAX_NODES> source_pressure{};
	std::array<float, MAX_SOURCES> source_pending{};
	std::array<float, MAX_SOURCES> source_pending_width_scale{};
	std::array<float, MAX_SOURCES> source_pending_attack_ratio{};
	std::array<float, MAX_SOURCES> source_attack_state{};
	std::array<float, MAX_SOURCES> source_decay_state{};
	std::array<float, MAX_SOURCES> source_event_attack_coefficient{};
	std::array<float, MAX_SOURCES> source_event_decay_coefficient{};
	std::array<float, MAX_SOURCES> source_event_normalization{};
	std::array<std::uint32_t, MAX_SOURCES> source_event_silent_after_samples{};
	std::array<std::uint32_t, MAX_SOURCES> source_age_samples{};
	std::array<bool, MAX_SOURCES> source_active{};
	std::array<float, MAX_NODES> node_incident{};
	std::array<float, MAX_TAPS> tap_values{};
	std::size_t node_count = 0;
	std::size_t guide_count = 0;
	std::size_t source_count = 0;
	std::size_t tap_count = 0;
	std::size_t used_delay_samples = 0;
	bool configured = false;

	float read_from(GuideId guide);
	float read_to(GuideId guide);
	void write_sample(GuideId guide, float from, float to);
	bool queue_source_event(SourceId source, float pressure,
			float width_scale, float attack_ratio);
};

} // namespace vehicle_audio
