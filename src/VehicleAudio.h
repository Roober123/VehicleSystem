#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "Audio/FiringScheduler.h"
#include "Audio/OutputConditioner.h"
#include "Audio/WaveguideTopology.h"
#include "godot_cpp/classes/audio_stream_generator.hpp"
#include "godot_cpp/classes/audio_stream_generator_playback.hpp"
#include "godot_cpp/classes/audio_stream_player3d.hpp"
#include "godot_cpp/variant/packed_vector2_array.hpp"

namespace godot {

class Vehicle;
class VehicleAudioFiringData;
class VehicleAudioTopologyData;

/// Spatialized procedural exhaust player. All graph and firing resources are
/// copied into bounded native runtime state once during _ready().
class VehicleAudio : public AudioStreamPlayer3D {
	GDCLASS(VehicleAudio, AudioStreamPlayer3D);

	static constexpr int MIX_RATE = 48000;
	static constexpr int STAGING_FRAMES = 256;

	Vehicle *target = nullptr;
	Ref<VehicleAudioTopologyData> topology;
	Ref<VehicleAudioFiringData> firing;
	Ref<AudioStreamGenerator> generator;
	Ref<AudioStreamGeneratorPlayback> playback;
	PackedVector2Array staging;

	vehicle_audio::WaveguideNetwork network;
	vehicle_audio::FourStrokeFiringScheduler scheduler;
	vehicle_audio::OutputConditioner conditioner;
	std::array<std::string, vehicle_audio::MAX_WAVEGUIDE_NODES> node_names{};
	std::array<std::string, vehicle_audio::MAX_WAVEGUIDES> guide_names{};
	std::array<std::string, vehicle_audio::MAX_WAVEGUIDE_SOURCES> source_names{};
	std::array<std::string, vehicle_audio::MAX_FIRING_PHASES> firing_names{};
	float crank_phase = 0.0f;
	std::uint32_t event_variation_index = 0;
	bool audio_enabled = false;
	bool setup_reported = false;

	bool compile_runtime();
	void disable_audio(const char *reason);

protected:
	static void _bind_methods();

public:
	VehicleAudio() = default;
	~VehicleAudio() override = default;

	void _ready() override;
	void _exit_tree() override;
	void _process(double delta) override;

	void set_target(Vehicle *value) { target = value; }
	Vehicle *get_target() const { return target; }
	void set_topology(const Ref<VehicleAudioTopologyData> &value) { topology = value; }
	Ref<VehicleAudioTopologyData> get_topology() const { return topology; }
	void set_firing(const Ref<VehicleAudioFiringData> &value) { firing = value; }
	Ref<VehicleAudioFiringData> get_firing() const { return firing; }
	bool is_audio_enabled() const { return audio_enabled; }
};

} // namespace godot
