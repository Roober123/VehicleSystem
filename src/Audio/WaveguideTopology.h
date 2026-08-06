#pragma once

#include "Audio/Waveguide.h"

namespace vehicle_audio {

constexpr float MAX_JUNCTION_NONLINEARITY = 1000.0f;

/// Bounded named graph authoring helper.  Names are non-owning pointers (the
/// usual use is string literals or vehicle-owned immutable data); IDs returned
/// by add_* are stable for the lifetime of the builder.
class WaveguideTopologyBuilder {
	WaveguideTopology topology;

	NodeId resolve_node(const char *name) const;
	SourceId resolve_source(const char *name) const;
	TapId resolve_tap(const char *name) const;
	bool valid_name(const char *name) const;

public:
	WaveguideTopologyBuilder() = default;

	void clear();

	NodeId add_junction(const char *name, float nonlinearity = 0.0f);
	NodeId add_reflection_boundary(const char *name, float reflection);

	GuideId add_guide(const char *name, NodeId from, NodeId to,
			float length_meters, float area, float loss_per_meter = 0.0f,
			float high_frequency_loss_per_meter = 0.0f);
	GuideId add_guide(const char *name, const char *from_name,
			const char *to_name, float length_meters, float area,
			float loss_per_meter = 0.0f,
			float high_frequency_loss_per_meter = 0.0f);

	SourceId add_source(const char *name, NodeId node, float gain = 1.0f,
			float pulse_width_ms = DEFAULT_PULSE_WIDTH_MS);
	SourceId add_source(const char *name, const char *node_name,
			float gain = 1.0f,
			float pulse_width_ms = DEFAULT_PULSE_WIDTH_MS);
	TapId add_tap(const char *name, NodeId node, float gain = 1.0f);
	TapId add_tap(const char *name, const char *node_name, float gain = 1.0f);
	TapId add_tap(const char *name, NodeId node, TapMode mode,
			float gain = 1.0f);
	TapId add_tap(const char *name, const char *node_name, TapMode mode,
			float gain = 1.0f);
	TapId add_boundary_radiation_tap(const char *name, NodeId node,
			float gain = 1.0f);
	TapId add_boundary_radiation_tap(const char *name, const char *node_name,
			float gain = 1.0f);

	NodeId find_node(const char *name) const { return resolve_node(name); }
	GuideId find_guide(const char *name) const;
	SourceId find_source(const char *name) const { return resolve_source(name); }
	TapId find_tap(const char *name) const { return resolve_tap(name); }

	const WaveguideTopology &description() const { return topology; }
	/// Validate and copy into a topology value.  This overload is useful to
	/// callers which want setup validation before constructing the runtime.
	bool build(WaveguideTopology &out) const;
	bool build(WaveguideNetwork &out,
			float sample_rate = DEFAULT_SAMPLE_RATE,
			float propagation_speed = DEFAULT_PROPAGATION_SPEED) const;
};

} // namespace vehicle_audio
