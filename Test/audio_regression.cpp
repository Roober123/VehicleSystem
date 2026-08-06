#include "audio_regression.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numeric>

#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/core/memory.hpp"
#include "godot_cpp/variant/packed_float32_array.hpp"
#include "godot_cpp/variant/packed_string_array.hpp"
#include "godot_cpp/variant/typed_array.hpp"
#include "godot_cpp/variant/utility_functions.hpp"

#include "Audio/FiringScheduler.h"
#include "Audio/OutputConditioner.h"
#include "Audio/WaveguideTopology.h"
#include "Resources/vehicle_audio_data.h"

namespace godot {
namespace {

using vehicle_audio::FourStrokeFiringScheduler;
using vehicle_audio::FiringEvent;
using vehicle_audio::GuideId;
using vehicle_audio::NodeId;
using vehicle_audio::OutputConditioner;
using vehicle_audio::SourceId;
using vehicle_audio::TapId;
using vehicle_audio::WaveguideNetwork;
using vehicle_audio::WaveguideTopologyBuilder;

constexpr float kEpsilon = 1.0e-5f;
// Use an explicit test clock so one metre is exactly one sample of travel;
// production defaults remain 48 kHz / 343 m/s.
constexpr float kTestSampleRate = 1000.0f;
constexpr float kTestPropagationSpeed = 1000.0f;

bool near(float actual, float expected, float tolerance = kEpsilon) {
	return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
}

bool build_for_test(const WaveguideTopologyBuilder &builder,
		WaveguideNetwork &network) {
	return builder.build(network, kTestSampleRate, kTestPropagationSpeed);
}

struct TestState {
	int failures = 0;

	void expect(bool condition, const char *label) {
		if (condition)
			return;
		++failures;
		UtilityFunctions::printerr("[audio-regression] FAIL: ", label);
	}

	void expect_near(float actual, float expected, const char *label,
			float tolerance = kEpsilon) {
		if (near(actual, expected, tolerance))
			return;
		++failures;
		UtilityFunctions::printerr("[audio-regression] FAIL: ", label,
				" actual=", actual, " expected=", expected);
	}
};

void test_topology_bounds(TestState &state) {
	WaveguideTopologyBuilder nodes;
	std::array<NodeId, vehicle_audio::MAX_WAVEGUIDE_NODES> node_ids{};
	for (std::size_t i = 0; i < node_ids.size(); ++i) {
		// The builder intentionally stores non-owning names.  Keep stable names
		// for this bounded check by using literals for the ordinary path below.
		// A duplicate-name check is covered separately; capacity is asserted by
		// adding the fixed set of literal names.
		static const char *const names[vehicle_audio::MAX_WAVEGUIDE_NODES] = {
			"n00", "n01", "n02", "n03", "n04", "n05", "n06", "n07",
			"n08", "n09", "n10", "n11", "n12", "n13", "n14", "n15",
			"n16", "n17", "n18", "n19", "n20", "n21", "n22", "n23",
			"n24", "n25", "n26", "n27", "n28", "n29", "n30", "n31"};
		node_ids[i] = nodes.add_junction(names[i]);
	}
	state.expect(node_ids.back() != vehicle_audio::INVALID_NODE,
			"Topology builder accepts exactly the bounded node capacity");
	state.expect(nodes.add_junction("overflow") == vehicle_audio::INVALID_NODE,
			"Topology builder rejects a node beyond the bounded capacity");
	state.expect(nodes.add_junction("n00") == vehicle_audio::INVALID_NODE,
			"Topology builder rejects duplicate node names");
	state.expect(nodes.add_junction("") == vehicle_audio::INVALID_NODE,
			"Topology builder rejects empty node names");

	WaveguideTopologyBuilder guides;
	const NodeId from = guides.add_junction("from");
	const NodeId to = guides.add_junction("to");
	std::array<char, vehicle_audio::MAX_WAVEGUIDES * 5> guide_names{};
	for (std::size_t i = 0; i < vehicle_audio::MAX_WAVEGUIDES; ++i) {
		char *name = guide_names.data() + i * 5;
		std::snprintf(name, 5, "g%03zu", i);
		state.expect(guides.add_guide(name, from, to, 1, 1.0f) !=
				vehicle_audio::INVALID_GUIDE,
			"Topology builder accepts guides up to the bounded capacity");
	}
	state.expect(guides.add_guide("overflow", from, to, 1, 1.0f) ==
				vehicle_audio::INVALID_GUIDE,
			"Topology builder rejects a guide beyond the bounded capacity");

	WaveguideTopologyBuilder sources;
	const NodeId source_node = sources.add_junction("source");
	const NodeId source_other = sources.add_junction("other");
	sources.add_guide("source-guide", source_node, source_other, 1, 1.0f);
	for (std::size_t i = 0; i < vehicle_audio::MAX_WAVEGUIDE_SOURCES; ++i) {
		char *name = guide_names.data() + i * 5;
		std::snprintf(name, 5, "s%03zu", i);
		state.expect(sources.add_source(name, source_node) !=
				vehicle_audio::INVALID_SOURCE,
			"Topology builder accepts sources up to the bounded capacity");
	}
	state.expect(sources.add_source("source-overflow", source_node) ==
				vehicle_audio::INVALID_SOURCE,
			"Topology builder rejects a source beyond the bounded capacity");

	WaveguideTopologyBuilder taps;
	const NodeId tap_node = taps.add_junction("tap");
	const NodeId tap_other = taps.add_junction("tap-other");
	taps.add_guide("tap-guide", tap_node, tap_other, 1, 1.0f);
	for (std::size_t i = 0; i < vehicle_audio::MAX_WAVEGUIDE_TAPS; ++i) {
		char *name = guide_names.data() + i * 5;
		std::snprintf(name, 5, "t%03zu", i);
		state.expect(taps.add_tap(name, tap_node) != vehicle_audio::INVALID_TAP,
				"Topology builder accepts taps up to the bounded capacity");
	}
	state.expect(taps.add_tap("tap-overflow", tap_node) ==
				vehicle_audio::INVALID_TAP,
			"Topology builder rejects a tap beyond the bounded capacity");
}

void test_delay_junction_attenuation_and_render(TestState &state) {
	WaveguideTopologyBuilder builder;
	const NodeId source_node = builder.add_junction("source");
	const NodeId tap_node = builder.add_junction("tap");
	const GuideId guide = builder.add_guide("delay", source_node, tap_node, 3.0f,
			1.0f, 0.25f);
	const SourceId source = builder.add_source("routed-source", source_node, 1.5f);
	const TapId source_tap = builder.add_tap("source-tap", source_node);
	const TapId destination_tap = builder.add_tap("destination-tap", tap_node);
	WaveguideNetwork network;
	state.expect(guide != vehicle_audio::INVALID_GUIDE &&
				source != vehicle_audio::INVALID_SOURCE &&
				source_tap != vehicle_audio::INVALID_TAP &&
				destination_tap != vehicle_audio::INVALID_TAP &&
				build_for_test(builder, network),
			"Waveguide network configures a delayed attenuated source route");
	state.expect(network.inject(source, 1.0f),
			"Waveguide network accepts a finite routed source injection");
	float output[4 * 2]{};
	state.expect(network.render_all(output, 4),
			"Interleaved tap rendering accepts a caller-owned output buffer");
	state.expect(output[0] > 0.0f && output[1] == 0.0f,
			"Source tap observes the shaped source pulse on the injection sample");
	state.expect(near(output[3], 0.0f) && near(output[5], 0.0f),
			"Integer guide delay keeps the destination silent before its exact sample");
	const float expected_traversal_loss = std::exp(-0.25f * 3.0f);
	state.expect(near(network.get_guide_delay_samples(guide), 3u) &&
				near(network.get_guide_traversal_loss(guide), expected_traversal_loss),
			"Physical guide length derives an integer delay and exponential traversal loss");
	state.expect(output[7] > 0.0f &&
				near(output[7], output[0] * 2.0f * expected_traversal_loss, 1.0e-4f),
			"Guide attenuation and terminal junction pressure are applied at exact delay");
	state.expect(network.get_used_delay_samples() == 6,
			"Used delay storage counts both directions exactly");
	state.expect(!network.inject(vehicle_audio::INVALID_SOURCE, 1.0f) &&
				!network.inject(source, std::numeric_limits<float>::quiet_NaN()),
			"Source injection rejects invalid IDs and non-finite pressure");
}

void test_equal_area_junction(TestState &state) {
	WaveguideTopologyBuilder builder;
	const NodeId source_node = builder.add_junction("source");
	const NodeId junction = builder.add_junction("junction");
	const NodeId terminal = builder.add_junction("terminal");
	builder.add_guide("inlet", source_node, junction, 1, 1.0f);
	builder.add_guide("outlet", junction, terminal, 1, 1.0f);
	const SourceId source = builder.add_source("pulse", source_node);
	builder.add_tap("junction-tap", junction);
	builder.add_tap("terminal-tap", terminal);
	WaveguideNetwork network;
	state.expect(build_for_test(builder, network) && network.inject(source, 1.0f),
			"Equal-area junction topology configures and accepts a pulse");
	float output[3 * 2]{};
	network.render_all(output, 3);
	state.expect(near(output[0], 0.0f) && near(output[1], 0.0f),
			"Equal-area junction starts with no incident wave");
	state.expect(output[2] > 0.0f && near(output[3], 0.0f),
			"Equal-area junction pressure transmits the shaped incident pulse");
	state.expect(output[5] > 0.0f,
			"Equal-area junction transmits to the second guide without gain bias");
}

void test_junction_nonlinearity(TestState &state) {
	WaveguideTopologyBuilder linear_builder;
	const NodeId linear_source = linear_builder.add_junction("linear-source");
	const NodeId linear_junction = linear_builder.add_junction("linear-junction", 0.0f);
	const NodeId linear_terminal =
			linear_builder.add_reflection_boundary("linear-terminal", 0.0f);
	linear_builder.add_guide("linear-inlet", linear_source, linear_junction,
			1.0f, 1.0f);
	linear_builder.add_guide("linear-outlet", linear_junction, linear_terminal,
			1.0f, 1.0f);
	const SourceId linear_id = linear_builder.add_source("linear-pulse",
			linear_source, 1.0f, 2.0f);
	linear_builder.add_tap("linear-source-tap", linear_source);
	linear_builder.add_tap("linear-junction-tap", linear_junction);
	WaveguideNetwork linear_network;
	state.expect(build_for_test(linear_builder, linear_network) &&
				linear_network.inject(linear_id, 1.0f),
			"Zero-nonlinearity junction accepts the reference pulse");
	float linear_trace[16 * 2]{};
	state.expect(linear_network.render_all(linear_trace, 16),
			"Zero-nonlinearity junction renders the reference trace");
	bool exact_linear = true;
	for (std::size_t frame = 0; frame + 1 < 16; ++frame)
		exact_linear = exact_linear &&
				near(linear_trace[(frame + 1) * 2 + 1], linear_trace[frame * 2],
						1.0e-5f);
	state.expect(exact_linear,
			"Zero junction nonlinearity preserves the exact linear wave response");

	WaveguideTopologyBuilder compressed_builder;
	const NodeId compressed_source = compressed_builder.add_junction("compressed-source");
	const NodeId compressed_junction =
			compressed_builder.add_junction("compressed-junction", 0.5f);
	const NodeId compressed_terminal =
			compressed_builder.add_reflection_boundary("compressed-terminal", 0.0f);
	compressed_builder.add_guide("compressed-inlet", compressed_source,
			compressed_junction, 1.0f, 1.0f);
	compressed_builder.add_guide("compressed-outlet", compressed_junction,
			compressed_terminal, 1.0f, 1.0f);
	const SourceId compressed_id = compressed_builder.add_source("compressed-pulse",
			compressed_source, 1.0f, 2.0f);
	compressed_builder.add_tap("compressed-junction-tap", compressed_junction);
	WaveguideNetwork compressed_network;
	state.expect(build_for_test(compressed_builder, compressed_network) &&
				compressed_network.inject(compressed_id, 1.0f),
			"Positive junction nonlinearity accepts the reference pulse");
	float compressed_trace[16]{};
	state.expect(compressed_network.render(compressed_trace, 16),
			"Positive junction nonlinearity renders the bounded trace");
	float compressed_peak = 0.0f;
	bool compressed_finite = true;
	for (float sample : compressed_trace) {
		compressed_peak = std::max(compressed_peak, std::abs(sample));
		compressed_finite = compressed_finite && std::isfinite(sample);
	}
	state.expect(compressed_finite && compressed_peak > 0.0f &&
			compressed_peak < 1.0f,
			"Positive junction nonlinearity remains finite and bounded below linear peak");
}

void test_reflection_boundaries(TestState &state) {
	struct BoundaryCase {
		float reflection;
		const char *label;
	};
	const BoundaryCase cases[] = {
		{-1.0f, "open"},
		{0.0f, "matched"},
		{1.0f, "closed"},
	};
	float radiation_peak[std::size(cases)]{};
	for (const BoundaryCase &test_case : cases) {
		WaveguideTopologyBuilder builder;
		const NodeId source_node = builder.add_junction("source");
		const NodeId boundary =
				builder.add_reflection_boundary("boundary", test_case.reflection);
		builder.add_guide("pipe", source_node, boundary, 1, 1.0f);
		const SourceId source = builder.add_source("pulse", source_node);
		const TapId boundary_tap = builder.add_tap("boundary-tap", boundary);
		const TapId radiation_tap =
				builder.add_boundary_radiation_tap("radiation-tap", boundary);
		WaveguideNetwork network;
		state.expect(build_for_test(builder, network) && network.inject(source, 1.0f),
				"Reflection boundary topology configures for each coefficient");
		float output[8 * 2]{};
		state.expect(boundary_tap != vehicle_audio::INVALID_TAP &&
					radiation_tap != vehicle_audio::INVALID_TAP &&
					network.render_all(output, 8),
				"Reflection boundary exposes node and radiation taps");
		float node_peak = 0.0f;
		float radiation_max = 0.0f;
		for (std::size_t frame = 0; frame < 8; ++frame) {
			node_peak = std::max(node_peak, std::abs(output[frame * 2]));
			radiation_max = std::max(radiation_max,
					std::abs(output[frame * 2 + 1]));
		}
		radiation_peak[&test_case - cases] = radiation_max;
		char label[96]{};
		std::snprintf(label, sizeof(label), "%s boundary has finite node pressure", test_case.label);
		state.expect(std::isfinite(node_peak), label);
	}
	state.expect(radiation_peak[0] > 0.0f &&
				radiation_peak[0] > radiation_peak[1] * 1.8f &&
				radiation_peak[0] < radiation_peak[1] * 2.2f,
			"Open radiation is approximately twice matched radiation");
	state.expect(radiation_peak[1] > 0.0f && radiation_peak[2] < kEpsilon,
			"Matched radiation passes incident pressure while closed radiation is zero");

	WaveguideTopologyBuilder isolated_boundary;
	const NodeId isolated =
			isolated_boundary.add_reflection_boundary("isolated", -1.0f);
	const NodeId connected_source = isolated_boundary.add_junction("source");
	const NodeId connected_tap = isolated_boundary.add_junction("tap");
	isolated_boundary.add_guide("connected-guide", connected_source, connected_tap,
			1.0f, 1.0f);
	isolated_boundary.add_source("pulse", connected_source);
	isolated_boundary.add_tap("output", connected_tap);
	WaveguideNetwork rejected;
	state.expect(isolated != vehicle_audio::INVALID_NODE &&
				!build_for_test(isolated_boundary, rejected),
			"Reflection boundaries with zero incident guides are rejected by degree validation");
}

void test_audio_resources(TestState &state) {
	Ref<VehicleAudioNodeData> node = memnew(VehicleAudioNodeData);
	node->set_name("collector");
	node->set_node_type(VehicleAudioNodeData::REFLECTION_BOUNDARY);
	node->set_reflection(-1.0);
	state.expect(node->get_name() == "collector" &&
				node->get_node_type() == VehicleAudioNodeData::REFLECTION_BOUNDARY &&
				near(static_cast<float>(node->get_reflection()), -1.0f),
			"VehicleAudioNodeData preserves boundary authoring fields");

	Ref<VehicleAudioGuideData> guide = memnew(VehicleAudioGuideData);
	guide->set_name("runner");
	guide->set_from("cylinder_1");
	guide->set_to("collector");
	guide->set_length_meters(real_t{0.25});
	guide->set_area(real_t{0.5});
	guide->set_loss_per_meter(real_t{0.75});
	state.expect(guide->get_name() == "runner" && guide->get_from() == "cylinder_1" &&
				guide->get_to() == "collector" &&
				near(static_cast<float>(guide->get_length_meters()), 0.25f) &&
				near(static_cast<float>(guide->get_area()), 0.5f) &&
				near(static_cast<float>(guide->get_loss_per_meter()), 0.75f),
			"VehicleAudioGuideData preserves physical geometry and loss fields");

	Ref<VehicleAudioTopologyData> topology = memnew(VehicleAudioTopologyData);
	TypedArray<VehicleAudioNodeData> nodes;
	nodes.push_back(node);
	TypedArray<VehicleAudioGuideData> guides;
	guides.push_back(guide);
	PackedStringArray source_names;
	source_names.push_back("cylinder_1");
	PackedStringArray source_nodes;
	source_nodes.push_back("collector");
	PackedFloat32Array source_gains;
	source_gains.push_back(0.75f);
	topology->set_nodes(nodes);
	topology->set_guides(guides);
	topology->set_source_names(source_names);
	topology->set_source_nodes(source_nodes);
	topology->set_source_gains(source_gains);
	topology->set_output_node("collector");
	topology->set_output_gain(real_t{0.5});
	state.expect(topology->get_nodes().size() == 1 && topology->get_guides().size() == 1 &&
				topology->get_source_names().size() == 1 &&
				topology->get_source_nodes()[0] == "collector" &&
				near(topology->get_source_gains()[0], 0.75f) &&
				topology->get_output_node() == "collector" &&
				near(static_cast<float>(topology->get_output_gain()), 0.5f),
			"VehicleAudioTopologyData preserves runner/source/output mappings");

	Ref<VehicleAudioFiringData> firing = memnew(VehicleAudioFiringData);
	const PackedFloat32Array phases = firing->get_phases_degrees();
	const PackedStringArray firing_sources = firing->get_source_names();
	const PackedFloat32Array firing_gains = firing->get_gains();
	state.expect(phases.size() == 4 && firing_sources.size() == 4 &&
				firing_gains.size() == 4 && near(phases[0], 0.0f) &&
				near(phases[1], 180.0f) && near(phases[2], 360.0f) &&
				near(phases[3], 540.0f) && firing_sources[0] == "cylinder_1" &&
				firing_sources[1] == "cylinder_3" &&
				firing_sources[2] == "cylinder_4" &&
				firing_sources[3] == "cylinder_2" && near(firing_gains[0], 1.0f) &&
				near(firing_gains[3], 1.0f),
			"VehicleAudioFiringData defaults to authored 1-3-4-2 phases and gains");
}

void test_topology_replacement_and_disconnected_subgraphs(TestState &state) {
	WaveguideTopologyBuilder valid;
	const NodeId source_node = valid.add_junction("source");
	const NodeId tap_node = valid.add_junction("tap");
	valid.add_guide("guide", source_node, tap_node, 1, 1.0f);
	const SourceId source = valid.add_source("source-id", source_node);
	valid.add_tap("tap-id", tap_node);
	WaveguideNetwork network;
	state.expect(build_for_test(valid, network),
			"Baseline topology configures before replacement attempt");

	WaveguideTopologyBuilder malformed;
	const NodeId malformed_source = malformed.add_junction("source");
	const NodeId malformed_tap = malformed.add_junction("tap");
	malformed.add_source("source-id", malformed_source);
	malformed.add_tap("tap-id", malformed_tap);
	state.expect(!network.configure(malformed.description(), kTestSampleRate,
				kTestPropagationSpeed) &&
				network.is_configured(),
			"Failed topology replacement leaves the prior network configured");
	state.expect(network.inject(source, 1.0f),
			"Prior source remains usable after failed replacement");
	float output[2]{};
	state.expect(network.render(output, 2),
			"Prior tap remains renderable after failed replacement");
	state.expect(output[1] > 0.0f,
			"Failed topology replacement is atomic for runtime propagation");

	WaveguideTopologyBuilder disconnected;
	const NodeId first = disconnected.add_junction("first");
	const NodeId second = disconnected.add_junction("second");
	const NodeId independent_source = disconnected.add_junction("independent-source");
	const NodeId independent_tap = disconnected.add_junction("independent-tap");
	disconnected.add_guide("first-guide", first, second, 1, 1.0f);
	disconnected.add_guide("independent-guide", independent_source, independent_tap,
			1, 1.0f);
	const SourceId independent_id =
			disconnected.add_source("independent", independent_source);
	disconnected.add_tap("independent-output", independent_tap);
	WaveguideNetwork independent_network;
	state.expect(build_for_test(disconnected, independent_network) &&
				independent_network.inject(independent_id, 1.0f),
			"Disconnected but independently-used subgraphs are accepted");
	float independent_output[2]{};
	state.expect(independent_network.render(independent_output, 2) &&
				independent_output[1] > 0.0f,
			"Independently-used disconnected subgraph propagates normally");
}

void test_pulse_shape_and_decay(TestState &state) {
	WaveguideTopologyBuilder builder;
	const NodeId source_node = builder.add_junction("pulse-source");
	const NodeId sink_node = builder.add_reflection_boundary("pulse-sink", 0.0f);
	const GuideId guide = builder.add_guide("pulse-guide", source_node, sink_node,
			1.0f, 1.0f);
	const SourceId source = builder.add_source("pulse-source-id", source_node,
			1.0f, 4.0f);
	const TapId tap = builder.add_tap("pulse-tap", source_node);
	WaveguideNetwork network;
	state.expect(guide != vehicle_audio::INVALID_GUIDE &&
				tap != vehicle_audio::INVALID_TAP && build_for_test(builder, network) &&
				network.get_source_pulse_width_ms(source) == 4.0f &&
				network.inject(source, 1.0f),
			"Pulse source compiles an authored multi-sample width");
	float output[40]{};
	state.expect(network.render(output, std::size(output)),
			"Pulse source renders into a bounded caller-owned trace");
	float peak = 0.0f;
	for (float sample : output)
		peak = std::max(peak, std::abs(sample));
	state.expect(output[0] > 0.0f && output[1] > 0.0f &&
				std::abs(output[0] - output[1]) > 1.0e-3f,
			"Combustion pulse occupies multiple samples with a shaped envelope");
	state.expect(peak > 0.9f && peak < 1.1f,
			"Combustion pulse peak is normalized near unit pressure");
	bool tail_silent = true;
	for (std::size_t i = 32; i < std::size(output); ++i)
		tail_silent = tail_silent && std::abs(output[i]) < 5.0e-3f;
	state.expect(tail_silent,
			"Combustion pulse is near silent within eight authored widths");
}

void test_high_frequency_loss(TestState &state) {
	WaveguideTopologyBuilder zero_builder;
	const NodeId zero_source = zero_builder.add_junction("zero-source");
	const NodeId zero_sink = zero_builder.add_junction("zero-sink");
	const GuideId zero_guide = zero_builder.add_guide("zero-guide", zero_source,
			zero_sink, 1.0f, 1.0f, 0.0f, 0.0f);
	const SourceId zero_id = zero_builder.add_source("zero-pulse", zero_source,
			1.0f, 2.0f);
	zero_builder.add_tap("zero-tap", zero_sink);
	WaveguideNetwork zero_network;
	state.expect(build_for_test(zero_builder, zero_network) &&
				zero_guide != vehicle_audio::INVALID_GUIDE &&
				near(zero_network.get_guide_high_frequency_alpha(zero_guide), 1.0f),
			"Zero high-frequency loss preserves identity propagation");

	WaveguideTopologyBuilder filtered_builder;
	const NodeId filtered_source = filtered_builder.add_junction("filtered-source");
	const NodeId filtered_sink = filtered_builder.add_junction("filtered-sink");
	const GuideId filtered_guide = filtered_builder.add_guide("filtered-guide",
			filtered_source, filtered_sink, 1.0f, 1.0f, 0.0f, 0.75f);
	const SourceId filtered_id = filtered_builder.add_source("filtered-pulse",
			filtered_source, 1.0f, 2.0f);
	filtered_builder.add_tap("filtered-tap", filtered_sink);
	WaveguideNetwork filtered_network;
	state.expect(build_for_test(filtered_builder, filtered_network) &&
				near(filtered_network.get_guide_high_frequency_alpha(filtered_guide),
						std::exp(-0.75f), 1.0e-4f),
			"Positive high-frequency loss compiles an exponential guide filter");

		float zero_alternating = 0.0f;
		float filtered_alternating = 0.0f;
		float zero_slow = 0.0f;
		float filtered_slow = 0.0f;
		float previous_zero_alt = 0.0f;
		float previous_filtered_alt = 0.0f;
		float previous_zero_slow = 0.0f;
		float previous_filtered_slow = 0.0f;
		for (std::size_t i = 0; i < 128; ++i) {
			const float alternating = i % 2 == 0 ? 1.0f : -1.0f;
			zero_network.inject(zero_id, alternating);
			filtered_network.inject(filtered_id, alternating);
			zero_network.process_sample();
			filtered_network.process_sample();
			const float zero_alt = zero_network.get_tap_value(0);
			const float filtered_alt = filtered_network.get_tap_value(0);
			if (i > 32) {
				zero_alternating += std::abs(zero_alt - previous_zero_alt);
				filtered_alternating += std::abs(filtered_alt - previous_filtered_alt);
			}
			previous_zero_alt = zero_alt;
			previous_filtered_alt = filtered_alt;
		}

		WaveguideTopologyBuilder slow_zero_builder;
		const NodeId slow_zero_source = slow_zero_builder.add_junction("slow-zero-source");
		const NodeId slow_zero_sink = slow_zero_builder.add_junction("slow-zero-sink");
		slow_zero_builder.add_guide("slow-zero-guide", slow_zero_source, slow_zero_sink,
				1.0f, 1.0f, 0.0f, 0.0f);
		const SourceId slow_zero_id = slow_zero_builder.add_source("slow-zero-pulse",
				slow_zero_source, 1.0f, 2.0f);
		slow_zero_builder.add_tap("slow-zero-tap", slow_zero_sink);
		WaveguideNetwork slow_zero_network;
		build_for_test(slow_zero_builder, slow_zero_network);
		WaveguideTopologyBuilder slow_filtered_builder;
		const NodeId slow_filtered_source = slow_filtered_builder.add_junction("slow-filtered-source");
		const NodeId slow_filtered_sink = slow_filtered_builder.add_junction("slow-filtered-sink");
		slow_filtered_builder.add_guide("slow-filtered-guide", slow_filtered_source,
				slow_filtered_sink, 1.0f, 1.0f, 0.0f, 0.75f);
		const SourceId slow_filtered_id = slow_filtered_builder.add_source("slow-filtered-pulse",
				slow_filtered_source, 1.0f, 2.0f);
		slow_filtered_builder.add_tap("slow-filtered-tap", slow_filtered_sink);
		WaveguideNetwork slow_filtered_network;
		build_for_test(slow_filtered_builder, slow_filtered_network);
		for (std::size_t i = 0; i < 128; ++i) {
			slow_zero_network.inject(slow_zero_id, 1.0f);
			slow_filtered_network.inject(slow_filtered_id, 1.0f);
			slow_zero_network.process_sample();
			slow_filtered_network.process_sample();
			const float slow_zero = slow_zero_network.get_tap_value(0);
			const float slow_filtered = slow_filtered_network.get_tap_value(0);
			if (i > 32) {
				zero_slow += std::abs(slow_zero - previous_zero_slow);
				filtered_slow += std::abs(slow_filtered - previous_filtered_slow);
			}
			previous_zero_slow = slow_zero;
			previous_filtered_slow = slow_filtered;
		}
	state.expect(zero_alternating > 0.0f && filtered_alternating < zero_alternating,
			"Positive high-frequency loss attenuates alternating content");
	state.expect(zero_slow > 0.0f && filtered_slow / zero_slow >
				filtered_alternating / zero_alternating,
			"Positive high-frequency loss preserves slow content more than alternating content");
}

void test_network_energy_decay(TestState &state) {
	WaveguideTopologyBuilder builder;
	const NodeId source_node = builder.add_junction("energy-source");
	const NodeId closed = builder.add_reflection_boundary("energy-closed", 1.0f);
	builder.add_guide("lossy-guide", source_node, closed, 1.0f, 1.0f, 0.35f);
	const SourceId source = builder.add_source("energy-pulse", source_node, 1.0f, 2.0f);
	const TapId tap = builder.add_tap("energy-tap", closed);
	WaveguideNetwork network;
	state.expect(tap != vehicle_audio::INVALID_TAP && build_for_test(builder, network) &&
				network.inject(source, 1.0f),
			"Lossy network accepts a finite post-injection pulse");
	float samples[128]{};
	state.expect(network.render(samples, std::size(samples), tap),
			"Lossy network renders a deterministic decay trace");
	float early_energy = 0.0f;
	float late_energy = 0.0f;
	for (std::size_t i = 0; i < std::size(samples); ++i) {
		if (i < 16)
			early_energy += std::abs(samples[i]);
		if (i >= 64)
			late_energy += std::abs(samples[i]);
	}
	state.expect(early_energy > 0.0f && late_energy < early_energy * 0.25f,
			"Post-injection network energy decays under broadband guide loss");
}

void test_output_conditioner(TestState &state) {
	vehicle_audio::OutputConditioner conditioner;
	float first = conditioner.process(1.0f);
	float latest = first;
	for (int i = 0; i < 4096; ++i)
		latest = conditioner.process(1.0f);
	state.expect(std::isfinite(first) && std::isfinite(latest) &&
				std::abs(latest) < 1.0e-4f,
			"Output conditioner removes a constant offset after transient settling");
	conditioner.reset();
	state.expect(near(conditioner.get_dc_previous_input(), 0.0f) &&
				near(conditioner.get_dc_previous_output(), 0.0f) &&
				near(conditioner.get_low_pass_previous(), 0.0f),
			"Output conditioner reset clears all bounded filter state");
}

void test_inject_event_envelope(TestState &state) {
	WaveguideTopologyBuilder builder;
	const NodeId source_node = builder.add_junction("event-source");
	const NodeId sink_node = builder.add_reflection_boundary("event-sink", 0.0f);
	builder.add_guide("event-guide", source_node, sink_node, 1.0f, 1.0f);
	const SourceId source = builder.add_source("event-source-id", source_node,
			1.0f, 6.0f);
	const TapId tap = builder.add_tap("event-tap", source_node);
	WaveguideNetwork network;
	state.expect(source != vehicle_audio::INVALID_SOURCE &&
				tap != vehicle_audio::INVALID_TAP && build_for_test(builder, network),
			"Event envelope topology configures at the deterministic clock");
	state.expect(near(network.get_source_pulse_width_ms(source), 6.0f),
			"Event shaping retains the authored source pulse width");

	using Envelope = std::array<float, 64>;
	// The test API needs a stable topology object for repeated runtime resets.
	// Rebuild it once so the first network configure above and every subsequent
	// case use identical authored geometry.
	vehicle_audio::WaveguideTopology authored;
	state.expect(builder.build(authored),
			"Event envelope topology exports a stable authored description");
	auto render_authored_event = [&](float rpm, float throttle, float variation,
			Envelope &output) {
			output.fill(0.0f);
			state.expect(network.configure(authored, kTestSampleRate,
					kTestPropagationSpeed),
					"Event envelope runtime reset is deterministic");
			state.expect(network.inject_event(source, 1.0f, rpm, throttle,
					variation),
					"Event envelope accepts finite RPM/throttle/variation inputs");
			state.expect(network.render(output.data(), output.size(), tap),
					"Event envelope renders a caller-owned multi-sample trace");
		};

	Envelope low{};
	Envelope high{};
	render_authored_event(800.0f, 0.0f, 1.0f, low);
	render_authored_event(8000.0f, 1.0f, 1.0f, high);
	float low_peak = 0.0f;
	float high_peak = 0.0f;
	float envelope_delta = 0.0f;
	bool finite = true;
	for (std::size_t i = 0; i < low.size(); ++i) {
		low_peak = std::max(low_peak, std::abs(low[i]));
		high_peak = std::max(high_peak, std::abs(high[i]));
		envelope_delta += std::abs(low[i] - high[i]);
		finite = finite && std::isfinite(low[i]) && std::isfinite(high[i]);
	}
	state.expect(finite && low_peak > 0.5f && high_peak > 0.5f,
			"Low/high event envelopes are finite and normalized to nonzero pressure");
	state.expect(envelope_delta > 1.0e-2f,
			"RPM and throttle produce measurably different multi-sample envelopes");

	Envelope base{};
	Envelope bounded_high{};
	Envelope bounded_low{};
	render_authored_event(2400.0f, 0.5f, 1.0f, base);
	render_authored_event(2400.0f, 0.5f, 100.0f, bounded_high);
	render_authored_event(2400.0f, 0.5f, -100.0f, bounded_low);
	float high_ratio_min = std::numeric_limits<float>::infinity();
	float high_ratio_max = 0.0f;
	float low_ratio_min = std::numeric_limits<float>::infinity();
	float low_ratio_max = 0.0f;
	for (std::size_t i = 0; i < base.size(); ++i) {
		if (std::abs(base[i]) < 1.0e-5f)
			continue;
		const float high_ratio = bounded_high[i] / base[i];
		const float low_ratio = bounded_low[i] / base[i];
		high_ratio_min = std::min(high_ratio_min, high_ratio);
		high_ratio_max = std::max(high_ratio_max, high_ratio);
		low_ratio_min = std::min(low_ratio_min, low_ratio);
		low_ratio_max = std::max(low_ratio_max, low_ratio);
	}
	state.expect(high_ratio_min >= 1.019f && high_ratio_max <= 1.021f &&
			low_ratio_min >= 0.979f && low_ratio_max <= 0.981f,
			"Event strength variation is clamped to a small bounded gain-only range");

	Envelope repeat{};
	render_authored_event(800.0f, 0.0f, 1.0f, repeat);
	float repeat_delta = 0.0f;
	for (std::size_t i = 0; i < repeat.size(); ++i)
		repeat_delta += std::abs(repeat[i] - low[i]);
	state.expect(repeat_delta < 1.0e-6f,
			"Resetting the event runtime reproduces the same deterministic envelope");
}

void test_deterministic_engine_traces(TestState &state) {
	struct TraceCase {
		float rpm;
		std::size_t expected_events;
		const char *label;
	};
	const TraceCase cases[] = {
		{850.0f, 56, "idle"},
		{2000.0f, 133, "2000 RPM"},
		{4000.0f, 266, "4000 RPM"},
	};
	constexpr float sample_rate = vehicle_audio::DEFAULT_SAMPLE_RATE;
	constexpr std::size_t frames = 96000; // Two seconds on the production clock.
	static const char *const source_node_names[] = {
		"trace-source-1", "trace-source-2", "trace-source-3", "trace-source-4"};
	static const char *const source_names[] = {
		"trace-cylinder-1", "trace-cylinder-2", "trace-cylinder-3",
		"trace-cylinder-4"};
	static const char *const guide_names[] = {
		"trace-guide-1", "trace-guide-2", "trace-guide-3", "trace-guide-4"};
	for (const TraceCase &trace_case : cases) {
		std::array<std::size_t, 2> event_counts{};
		std::array<std::array<std::size_t, 4>, 2> source_counts{};
		std::array<float, 2> amplitude_sums{};
		std::array<float, 2> waveform_energy{};
		std::array<float, 2> waveform_peak{};
		std::array<bool, 2> waveform_finite{true, true};
		std::array<bool, 2> render_ok{true, true};
		for (std::size_t throttle_case = 0; throttle_case < 2; ++throttle_case) {
			const float throttle = throttle_case == 0 ? 0.1f : 1.0f;
			FourStrokeFiringScheduler scheduler =
					FourStrokeFiringScheduler::default_four_stroke();
			WaveguideTopologyBuilder trace_builder;
			const std::array<NodeId, 4> source_nodes = {
				trace_builder.add_junction(source_node_names[0]),
				trace_builder.add_junction(source_node_names[1]),
				trace_builder.add_junction(source_node_names[2]),
				trace_builder.add_junction(source_node_names[3])};
			const NodeId collector = trace_builder.add_junction("trace-collector");
			const NodeId boundary =
				trace_builder.add_reflection_boundary("trace-boundary", 0.0f);
			for (std::size_t source_index = 0; source_index < source_nodes.size();
					source_index++) {
				trace_builder.add_guide(guide_names[source_index], source_nodes[source_index],
						collector, 1.0f, 1.0f, 0.02f);
				trace_builder.add_source(source_names[source_index], source_nodes[source_index],
						1.0f, 1.5f);
			}
			trace_builder.add_guide("trace-tail", collector, boundary, 1.0f, 1.0f,
					0.02f);
			const TapId trace_tap = trace_builder.add_tap("trace-output", collector);
			WaveguideNetwork trace_network;
			const bool configured = trace_tap != vehicle_audio::INVALID_TAP &&
					trace_builder.build(trace_network, sample_rate,
							kTestPropagationSpeed);
			state.expect(configured,
					"Engine trace waveguide configures at the deterministic sample rate");
			OutputConditioner conditioner;
			FiringEvent events[vehicle_audio::MAX_FIRING_PHASES]{};
			scheduler.advance(0.0f, events, std::size(events));
			float phase = 0.0f;
			for (std::size_t frame = 0; frame < frames; ++frame) {
				phase += trace_case.rpm * 6.0f / sample_rate;
				const std::size_t count = scheduler.advance(phase, events,
						std::size(events));
				event_counts[throttle_case] += count;
				for (std::size_t i = 0; i < count; ++i) {
					const SourceId source = events[i].source;
					if (source < source_counts[throttle_case].size())
						source_counts[throttle_case][source]++;
					amplitude_sums[throttle_case] += std::abs(events[i].amplitude);
					const float event_pressure = events[i].amplitude *
							(0.1f + 0.9f * throttle);
					render_ok[throttle_case] = trace_network.inject_event(source,
							event_pressure, trace_case.rpm, throttle, 1.0f) &&
							render_ok[throttle_case];
				}
				float raw_sample = 0.0f;
				const bool rendered = trace_network.render(&raw_sample, 1, trace_tap);
				render_ok[throttle_case] = rendered && render_ok[throttle_case];
				const float conditioned_sample = conditioner.process(raw_sample);
				waveform_finite[throttle_case] =
						std::isfinite(raw_sample) && std::isfinite(conditioned_sample) &&
						waveform_finite[throttle_case];
				waveform_energy[throttle_case] += std::abs(conditioned_sample);
				waveform_peak[throttle_case] = std::max(
						waveform_peak[throttle_case], std::abs(conditioned_sample));
			}
		}
		char label[128]{};
		std::snprintf(label, sizeof(label), "%s low-throttle trace has the firing fundamental",
				trace_case.label);
			state.expect(event_counts[0] == trace_case.expected_events, label);
		std::snprintf(label, sizeof(label), "%s full-throttle trace has no skipped or duplicated events",
				trace_case.label);
			state.expect(event_counts[1] == trace_case.expected_events, label);
		std::snprintf(label, sizeof(label), "%s routes every cylinder without skipped events",
				trace_case.label);
		state.expect(std::all_of(source_counts[0].begin(), source_counts[0].end(),
					[](std::size_t count) { return count > 0; }) &&
					std::all_of(source_counts[1].begin(), source_counts[1].end(),
							[](std::size_t count) { return count > 0; }), label);
		state.expect_near(amplitude_sums[1], amplitude_sums[0],
				"Low/full throttle traces preserve authored scheduler gains", 1.0e-3f);
		std::snprintf(label, sizeof(label), "%s low/full traces render finite bounded waveforms",
				trace_case.label);
		state.expect(render_ok[0] && render_ok[1] && waveform_finite[0] &&
					waveform_finite[1] && waveform_peak[0] < 0.95f &&
					waveform_peak[1] < 0.95f, label);
		std::snprintf(label, sizeof(label), "%s low/full traces produce nonzero conditioned energy",
				trace_case.label);
		state.expect(waveform_energy[0] > 1.0e-4f && waveform_energy[1] > 1.0e-4f,
				label);
		std::snprintf(label, sizeof(label), "%s full throttle waveform is stronger without saturation",
				trace_case.label);
		state.expect(waveform_energy[1] > waveform_energy[0] * 2.0f &&
					waveform_peak[1] > waveform_peak[0] * 1.5f &&
					waveform_peak[1] < 0.95f, label);
	}
}

void test_scheduler(TestState &state) {
	FourStrokeFiringScheduler scheduler =
			FourStrokeFiringScheduler::default_four_stroke();
	state.expect(scheduler.get_phase_count() == 4,
			"Default scheduler authors four cylinder phases");
	state.expect(near(scheduler.get_phase(0).crank_degrees, 0.0f) &&
				near(scheduler.get_phase(1).crank_degrees, 180.0f) &&
				near(scheduler.get_phase(2).crank_degrees, 360.0f) &&
				near(scheduler.get_phase(3).crank_degrees, 540.0f) &&
				scheduler.get_phase(0).source == 0 &&
				scheduler.get_phase(1).source == 2 &&
				scheduler.get_phase(2).source == 3 &&
				scheduler.get_phase(3).source == 1,
			"Default scheduler preserves 1-3-4-2 source routing");

		FiringEvent events[vehicle_audio::MAX_FIRING_PHASES]{};
	std::size_t fire_count = 0;
	scheduler.advance(1.0f, events, std::size(events));
	for (int step = 1; step <= 100; ++step)
		fire_count += scheduler.advance(1.0f + 72.0f * step, events,
					std::size(events));
	state.expect(fire_count == 40,
			"Four-cylinder scheduler emits 40 fires over one second at 1200 RPM");

	FourStrokeFiringScheduler authored;
	authored.append_phase(45.0f, 3, 0.5f);
	authored.append_phase(210.0f, 1, 2.0f);
	authored.append_phase(710.0f, 0, -1.0f);
	authored.append_phase(0.0f, 2, 4.0f);
	FiringEvent block[2]{};
	const std::size_t block_count = authored.collect_events(40.0f, 720.0f, block,
			std::size(block));
	state.expect(block_count == 4 && block[0].source == 3 &&
				block[1].source == 1 && near(block[0].amplitude, 0.5f) &&
				near(block[1].amplitude, 2.0f),
			"Scheduler reports all authored uneven phases across a block crossing");
	FiringEvent wrap[4]{};
	const std::size_t wrap_count = authored.collect_events(700.0f, 20.0f, wrap,
			std::size(wrap));
	state.expect(wrap_count == 2 && wrap[0].source == 0 && wrap[1].source == 2,
			"Scheduler emits authored source routing across a cycle wrap");
	state.expect(!authored.set_phase(vehicle_audio::MAX_FIRING_PHASES, 10.0f, 0) &&
				!authored.set_phase(0, 10.0f, vehicle_audio::MAX_WAVEGUIDE_SOURCES) &&
				!authored.set_phase(0, std::numeric_limits<float>::infinity(), 0),
			"Scheduler rejects phase, source, and finite-value bounds");
}

} // namespace

void AudioRegression::_bind_methods() {
	ClassDB::bind_method(D_METHOD("run"), &AudioRegression::run);
}

bool AudioRegression::run() {
	TestState state;
	test_topology_bounds(state);
	test_delay_junction_attenuation_and_render(state);
	test_equal_area_junction(state);
	test_junction_nonlinearity(state);
	test_reflection_boundaries(state);
	test_topology_replacement_and_disconnected_subgraphs(state);
	test_pulse_shape_and_decay(state);
	test_high_frequency_loss(state);
	test_network_energy_decay(state);
	test_output_conditioner(state);
	test_inject_event_envelope(state);
	test_deterministic_engine_traces(state);
	test_scheduler(state);
	test_audio_resources(state);
	if (state.failures != 0)
		UtilityFunctions::printerr("[audio-regression] ", state.failures,
				" assertion(s) failed");
	else
		UtilityFunctions::print("[audio-regression] all assertions passed");
	return state.failures == 0;
}

} // namespace godot
