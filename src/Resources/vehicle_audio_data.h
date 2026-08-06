#pragma once

#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/variant/packed_float32_array.hpp"
#include "godot_cpp/variant/packed_string_array.hpp"
#include "godot_cpp/variant/typed_array.hpp"

namespace godot {

class VehicleAudioNodeData : public Resource {
	GDCLASS(VehicleAudioNodeData, Resource);

	public:
	enum NodeType { JUNCTION = 0, REFLECTION_BOUNDARY = 1 };

	private:
	String name;
	int node_type = 0; // 0: junction, 1: reflection boundary
	real_t reflection = 0.0;
	real_t nonlinearity = 0.0;

protected:
	static void _bind_methods();

public:
	void set_name(const String &value) { name = value; }
	String get_name() const { return name; }
	void set_node_type(int value) { node_type = value; }
	int get_node_type() const { return node_type; }
	void set_reflection(real_t value) { reflection = value; }
	real_t get_reflection() const { return reflection; }
	void set_nonlinearity(real_t value) { nonlinearity = value; }
	real_t get_nonlinearity() const { return nonlinearity; }
};

class VehicleAudioGuideData : public Resource {
	GDCLASS(VehicleAudioGuideData, Resource);

	String name;
	String from;
	String to;
	real_t length_meters = 1.0;
	real_t area = 1.0;
	real_t loss_per_meter = 0.0;
	real_t high_frequency_loss_per_meter = 0.0;

protected:
	static void _bind_methods();

public:
	void set_name(const String &value) { name = value; }
	String get_name() const { return name; }
	void set_from(const String &value) { from = value; }
	String get_from() const { return from; }
	void set_to(const String &value) { to = value; }
	String get_to() const { return to; }
	void set_length_meters(real_t value) { length_meters = value; }
	real_t get_length_meters() const { return length_meters; }
	void set_area(real_t value) { area = value; }
	real_t get_area() const { return area; }
	void set_loss_per_meter(real_t value) { loss_per_meter = value; }
	real_t get_loss_per_meter() const { return loss_per_meter; }
	void set_high_frequency_loss_per_meter(real_t value) { high_frequency_loss_per_meter = value; }
	real_t get_high_frequency_loss_per_meter() const { return high_frequency_loss_per_meter; }
};

class VehicleAudioTopologyData : public Resource {
	GDCLASS(VehicleAudioTopologyData, Resource);

	TypedArray<VehicleAudioNodeData> nodes;
	TypedArray<VehicleAudioGuideData> guides;
	PackedStringArray source_names;
	PackedStringArray source_nodes;
	PackedFloat32Array source_gains;
	String output_node;
	real_t output_gain = 1.0;
	int output_tap_type = 0;

protected:
	static void _bind_methods();

public:
	void set_nodes(const TypedArray<VehicleAudioNodeData> &value) { nodes = value; }
	TypedArray<VehicleAudioNodeData> get_nodes() const { return nodes; }
	void set_guides(const TypedArray<VehicleAudioGuideData> &value) { guides = value; }
	TypedArray<VehicleAudioGuideData> get_guides() const { return guides; }
	void set_source_names(const PackedStringArray &value) { source_names = value; }
	PackedStringArray get_source_names() const { return source_names; }
	void set_source_nodes(const PackedStringArray &value) { source_nodes = value; }
	PackedStringArray get_source_nodes() const { return source_nodes; }
	void set_source_gains(const PackedFloat32Array &value) { source_gains = value; }
	PackedFloat32Array get_source_gains() const { return source_gains; }
	void set_output_node(const String &value) { output_node = value; }
	String get_output_node() const { return output_node; }
	void set_output_gain(real_t value) { output_gain = value; }
	real_t get_output_gain() const { return output_gain; }
	void set_output_tap_type(int value) { output_tap_type = value; }
	int get_output_tap_type() const { return output_tap_type; }
};

class VehicleAudioFiringData : public Resource {
	GDCLASS(VehicleAudioFiringData, Resource);

	PackedFloat32Array phases_degrees;
	PackedStringArray source_names;
	PackedFloat32Array gains;
	real_t pulse_width_ms = 1.5;

protected:
	static void _bind_methods();

public:
	VehicleAudioFiringData();

	void set_phases_degrees(const PackedFloat32Array &value) { phases_degrees = value; }
	PackedFloat32Array get_phases_degrees() const { return phases_degrees; }
	void set_source_names(const PackedStringArray &value) { source_names = value; }
	PackedStringArray get_source_names() const { return source_names; }
	void set_gains(const PackedFloat32Array &value) { gains = value; }
	PackedFloat32Array get_gains() const { return gains; }
	void set_pulse_width_ms(real_t value) { pulse_width_ms = value; }
	real_t get_pulse_width_ms() const { return pulse_width_ms; }
	void configure_default_four_stroke();
};

} // namespace godot

VARIANT_ENUM_CAST(godot::VehicleAudioNodeData::NodeType);
