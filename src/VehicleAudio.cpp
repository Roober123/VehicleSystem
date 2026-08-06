#include "VehicleAudio.h"

#include <algorithm>
#include <cmath>

#include "Resources/vehicle_audio_data.h"
#include "godot_cpp/classes/audio_stream.hpp"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/variant/typed_array.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "vehicle.h"

namespace godot {
namespace {

std::string to_utf8(const String &value) {
	const CharString encoded = value.utf8();
	return encoded.get_data() != nullptr ? std::string(encoded.get_data()) : std::string();
}

float event_strength_variation(std::uint32_t index) {
	// A tiny authored cycle keeps adjacent cylinders from being perfectly
	// identical without introducing RNG state or changing firing timing.
	static constexpr float pattern[] = {0.988f, 1.012f, 0.996f, 1.004f};
	return pattern[index % (sizeof(pattern) / sizeof(pattern[0]))];
}

} // namespace

void VehicleAudio::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_target", "vehicle"), &VehicleAudio::set_target);
	ClassDB::bind_method(D_METHOD("get_target"), &VehicleAudio::get_target);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "target", PROPERTY_HINT_NODE_TYPE,
			"Vehicle"), "set_target", "get_target");
	ClassDB::bind_method(D_METHOD("set_topology", "topology"), &VehicleAudio::set_topology);
	ClassDB::bind_method(D_METHOD("get_topology"), &VehicleAudio::get_topology);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "topology", PROPERTY_HINT_RESOURCE_TYPE,
			"VehicleAudioTopologyData"), "set_topology", "get_topology");
	ClassDB::bind_method(D_METHOD("set_firing", "firing"), &VehicleAudio::set_firing);
	ClassDB::bind_method(D_METHOD("get_firing"), &VehicleAudio::get_firing);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "firing", PROPERTY_HINT_RESOURCE_TYPE,
			"VehicleAudioFiringData"), "set_firing", "get_firing");
	ClassDB::bind_method(D_METHOD("is_audio_enabled"), &VehicleAudio::is_audio_enabled);
}

void VehicleAudio::disable_audio(const char *reason) {
	audio_enabled = false;
	stop();
	playback.unref();
	if (!setup_reported) {
		UtilityFunctions::printerr(String("VehicleAudio setup failed: ") + String(reason));
		setup_reported = true;
	}
}

bool VehicleAudio::compile_runtime() {
	if (topology.is_null() || firing.is_null()) {
		disable_audio("topology and firing resources are required");
		return false;
	}

	vehicle_audio::WaveguideTopologyBuilder builder;
	const TypedArray<VehicleAudioNodeData> authored_nodes = topology->get_nodes();
	if (authored_nodes.size() > static_cast<int64_t>(vehicle_audio::MAX_WAVEGUIDE_NODES)) {
		disable_audio("node capacity exceeded");
		return false;
	}
	for (int64_t i = 0; i < authored_nodes.size(); ++i) {
		Ref<VehicleAudioNodeData> node = authored_nodes[i];
		if (node.is_null()) {
			disable_audio("null node resource");
			return false;
		}
		node_names[i] = to_utf8(node->get_name());
		vehicle_audio::NodeId id = vehicle_audio::INVALID_NODE;
		if (node->get_node_type() == 0) {
			id = builder.add_junction(node_names[i].c_str(),
					static_cast<float>(node->get_nonlinearity()));
		} else if (node->get_node_type() == 1) {
			if (std::abs(static_cast<float>(node->get_nonlinearity())) > 1.0e-6f) {
				disable_audio("nonlinearity is only valid on junction nodes");
				return false;
			}
			id = builder.add_reflection_boundary(node_names[i].c_str(),
					static_cast<float>(node->get_reflection()));
		}
		if (id == vehicle_audio::INVALID_NODE) {
			disable_audio("invalid or duplicate node");
			return false;
		}
	}

	const TypedArray<VehicleAudioGuideData> authored_guides = topology->get_guides();
	if (authored_guides.size() > static_cast<int64_t>(vehicle_audio::MAX_WAVEGUIDES)) {
		disable_audio("guide capacity exceeded");
		return false;
	}
	for (int64_t i = 0; i < authored_guides.size(); ++i) {
		Ref<VehicleAudioGuideData> guide = authored_guides[i];
		if (guide.is_null()) {
			disable_audio("null guide resource");
			return false;
		}
		guide_names[i] = to_utf8(guide->get_name());
		const std::string from_name = to_utf8(guide->get_from());
		const std::string to_name = to_utf8(guide->get_to());
		const vehicle_audio::NodeId from = builder.find_node(from_name.c_str());
		const vehicle_audio::NodeId to = builder.find_node(to_name.c_str());
		if (builder.add_guide(guide_names[i].c_str(),
					from, to,
					static_cast<float>(guide->get_length_meters()),
					static_cast<float>(guide->get_area()),
					static_cast<float>(guide->get_loss_per_meter()),
					static_cast<float>(guide->get_high_frequency_loss_per_meter())) ==
				vehicle_audio::INVALID_GUIDE) {
			disable_audio("invalid guide endpoint or geometry");
			return false;
		}
	}

	const PackedStringArray authored_sources = topology->get_source_names();
	const PackedStringArray authored_source_nodes = topology->get_source_nodes();
	const PackedFloat32Array authored_source_gains = topology->get_source_gains();
	if (authored_sources.size() > static_cast<int64_t>(vehicle_audio::MAX_WAVEGUIDE_SOURCES) ||
			(authored_source_nodes.size() != 0 &&
					authored_source_nodes.size() != authored_sources.size())) {
		disable_audio("source capacity exceeded");
		return false;
	}
	for (int64_t i = 0; i < authored_sources.size(); ++i) {
		source_names[i] = to_utf8(authored_sources[i]);
		const std::string source_node_name = authored_source_nodes.size() == authored_sources.size()
				? to_utf8(authored_source_nodes[i])
				: source_names[i];
		const vehicle_audio::NodeId source_node = builder.find_node(source_node_name.c_str());
		const float gain = authored_source_gains.size() > i ? authored_source_gains[i] : 1.0f;
		const float pulse_width = static_cast<float>(firing->get_pulse_width_ms());
		if (builder.add_source(source_names[i].c_str(), source_node, gain,
				pulse_width) ==
				vehicle_audio::INVALID_SOURCE) {
			disable_audio("invalid source mapping");
			return false;
		}
	}
	const std::string output_name = to_utf8(topology->get_output_node());
	const int output_tap_type = topology->get_output_tap_type();
	if (output_name.empty() || (output_tap_type != 0 && output_tap_type != 1)) {
		disable_audio("invalid output tap type");
		return false;
	}
	const vehicle_audio::TapMode tap_mode = output_tap_type == 1
			? vehicle_audio::TapMode::BOUNDARY_RADIATION
			: vehicle_audio::TapMode::NODE_PRESSURE;
	if (builder.add_tap("vehicle_audio_output", output_name.c_str(), tap_mode,
				static_cast<float>(topology->get_output_gain())) == vehicle_audio::INVALID_TAP) {
		disable_audio("invalid output node");
		return false;
	}
	if (!builder.build(network, static_cast<float>(MIX_RATE),
				vehicle_audio::DEFAULT_PROPAGATION_SPEED)) {
		disable_audio("topology validation failed");
		return false;
	}

	const PackedFloat32Array authored_phases = firing->get_phases_degrees();
	const PackedStringArray authored_firing_sources = firing->get_source_names();
	const PackedFloat32Array authored_firing_gains = firing->get_gains();
	if (authored_phases.size() == 0 || authored_phases.size() >
				static_cast<int64_t>(vehicle_audio::MAX_FIRING_PHASES) ||
			authored_firing_sources.size() != authored_phases.size() ||
			authored_firing_gains.size() != authored_phases.size()) {
		disable_audio("invalid firing phase arrays");
		return false;
	}
	scheduler.clear();
	for (int64_t i = 0; i < authored_phases.size(); ++i) {
		firing_names[i] = to_utf8(authored_firing_sources[i]);
		const vehicle_audio::SourceId source = builder.find_source(firing_names[i].c_str());
		if (source == vehicle_audio::INVALID_SOURCE ||
				!scheduler.set_phase(static_cast<std::size_t>(i), authored_phases[i], source,
						authored_firing_gains[i])) {
			disable_audio("firing phase references an unknown source");
			return false;
		}
	}

	generator.instantiate();
	if (generator.is_null()) {
		disable_audio("audio generator allocation failed");
		return false;
	}
	generator->set_mix_rate_mode(AudioStreamGenerator::MIX_RATE_CUSTOM);
	generator->set_mix_rate(static_cast<float>(MIX_RATE));
	generator->set_buffer_length(0.1f);
	set_stream(generator);
	play();
	playback = get_stream_playback();
	if (playback.is_null()) {
		disable_audio("audio generator playback unavailable");
		return false;
	}
	if (staging.resize(STAGING_FRAMES) != 0) {
		disable_audio("audio staging allocation failed");
		return false;
	}
	crank_phase = 0.0f;
	event_variation_index = 0;
	scheduler.reset(crank_phase);
	conditioner.reset();
	audio_enabled = true;
	return true;
}

void VehicleAudio::_ready() {
	if (Engine::get_singleton()->is_editor_hint())
		return;
	setup_reported = false;
	audio_enabled = false;
	if (target == nullptr)
		target = Object::cast_to<Vehicle>(get_parent());
	if (target == nullptr) {
		Node *parent = get_parent();
		if (parent != nullptr) {
			const TypedArray<Node> children = parent->get_children();
			for (int64_t i = 0; i < children.size(); ++i) {
				target = Object::cast_to<Vehicle>(children[i]);
				if (target != nullptr)
					break;
			}
		}
	}
	compile_runtime();
}

void VehicleAudio::_exit_tree() {
	// Teardown is a normal lifecycle transition, not a setup failure. Release
	// the playback before its generator/stream owner so Godot cannot retain a
	// stale generator playback across scene re-entry.
	audio_enabled = false;
	stop();
	if (playback.is_valid()) {
		playback->stop();
		playback->clear_buffer();
	}
	playback.unref();
	set_stream(Ref<AudioStream>());
	generator.unref();
	staging.clear();
	event_variation_index = 0;
}

void VehicleAudio::_process(double delta) {
	(void)delta;
	if (!audio_enabled || target == nullptr || playback.is_null() ||
			!playback->can_push_buffer(STAGING_FRAMES))
		return;
	const VehicleTelemetrySnapshot snapshot = target->get_telemetry_snapshot();
	const float rpm = std::isfinite(static_cast<float>(snapshot.engine_rpm))
			? std::max(0.0f, static_cast<float>(snapshot.engine_rpm))
			: 0.0f;
	const float throttle = std::isfinite(static_cast<float>(snapshot.engine_throttle))
			? std::clamp(static_cast<float>(snapshot.engine_throttle), 0.0f, 1.0f)
			: 0.0f;
	const float phase_step = rpm * 6.0f / static_cast<float>(MIX_RATE);
	const float pulse_scale = 0.1f + 0.9f * throttle;
	std::array<vehicle_audio::FiringEvent, vehicle_audio::MAX_FIRING_PHASES> events{};
	while (playback->can_push_buffer(STAGING_FRAMES)) {
		for (int i = 0; i < STAGING_FRAMES; ++i) {
			const std::size_t event_count = scheduler.advance(crank_phase,
					events.data(), events.size());
			for (std::size_t event_index = 0; event_index < event_count;
					event_index++) {
				const vehicle_audio::FiringEvent &event = events[event_index];
				network.inject_event(event.source, event.amplitude * pulse_scale, rpm, throttle,
						event_strength_variation(event_variation_index++));
			}
			const float conditioned = conditioner.process(network.process_sample());
			staging.set(i, Vector2(conditioned, conditioned));
			crank_phase += phase_step;
			if (crank_phase >= vehicle_audio::FOUR_STROKE_CYCLE_DEGREES)
				crank_phase -= vehicle_audio::FOUR_STROKE_CYCLE_DEGREES;
		}
		if (!playback->push_buffer(staging))
			break;
	}
}

} // namespace godot
