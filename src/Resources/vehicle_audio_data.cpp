#include "vehicle_audio_data.h"

namespace godot {

void VehicleAudioNodeData::_bind_methods() {
	BIND_ENUM_CONSTANT(JUNCTION);
	BIND_ENUM_CONSTANT(REFLECTION_BOUNDARY);
	ClassDB::bind_method(D_METHOD("set_name", "value"), &VehicleAudioNodeData::set_name);
	ClassDB::bind_method(D_METHOD("get_name"), &VehicleAudioNodeData::get_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "name"), "set_name", "get_name");
	ClassDB::bind_method(D_METHOD("set_node_type", "value"), &VehicleAudioNodeData::set_node_type);
	ClassDB::bind_method(D_METHOD("get_node_type"), &VehicleAudioNodeData::get_node_type);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "node_type", PROPERTY_HINT_ENUM,
			"Junction,ReflectionBoundary"), "set_node_type", "get_node_type");
	ClassDB::bind_method(D_METHOD("set_reflection", "value"), &VehicleAudioNodeData::set_reflection);
	ClassDB::bind_method(D_METHOD("get_reflection"), &VehicleAudioNodeData::get_reflection);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reflection", PROPERTY_HINT_RANGE,
			"-1,1,0.001"), "set_reflection", "get_reflection");
	ClassDB::bind_method(D_METHOD("set_nonlinearity", "value"),
			&VehicleAudioNodeData::set_nonlinearity);
	ClassDB::bind_method(D_METHOD("get_nonlinearity"),
			&VehicleAudioNodeData::get_nonlinearity);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "nonlinearity", PROPERTY_HINT_RANGE,
			"0,10,0.001,or_greater"), "set_nonlinearity", "get_nonlinearity");
}

void VehicleAudioGuideData::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_name", "value"), &VehicleAudioGuideData::set_name);
	ClassDB::bind_method(D_METHOD("get_name"), &VehicleAudioGuideData::get_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "name"), "set_name", "get_name");
	ClassDB::bind_method(D_METHOD("set_from", "value"), &VehicleAudioGuideData::set_from);
	ClassDB::bind_method(D_METHOD("get_from"), &VehicleAudioGuideData::get_from);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "from"), "set_from", "get_from");
	ClassDB::bind_method(D_METHOD("set_to", "value"), &VehicleAudioGuideData::set_to);
	ClassDB::bind_method(D_METHOD("get_to"), &VehicleAudioGuideData::get_to);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "to"), "set_to", "get_to");
	ClassDB::bind_method(D_METHOD("set_length_meters", "value"), &VehicleAudioGuideData::set_length_meters);
	ClassDB::bind_method(D_METHOD("get_length_meters"), &VehicleAudioGuideData::get_length_meters);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "length_meters", PROPERTY_HINT_RANGE,
			"0.001,100,0.001,or_greater"), "set_length_meters", "get_length_meters");
	ClassDB::bind_method(D_METHOD("set_area", "value"), &VehicleAudioGuideData::set_area);
	ClassDB::bind_method(D_METHOD("get_area"), &VehicleAudioGuideData::get_area);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "area", PROPERTY_HINT_RANGE,
			"0.000001,10,0.000001,or_greater"), "set_area", "get_area");
	ClassDB::bind_method(D_METHOD("set_loss_per_meter", "value"), &VehicleAudioGuideData::set_loss_per_meter);
	ClassDB::bind_method(D_METHOD("get_loss_per_meter"), &VehicleAudioGuideData::get_loss_per_meter);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "loss_per_meter", PROPERTY_HINT_RANGE,
			"0,100,0.001,or_greater"), "set_loss_per_meter", "get_loss_per_meter");
	ClassDB::bind_method(D_METHOD("set_high_frequency_loss_per_meter", "value"),
			&VehicleAudioGuideData::set_high_frequency_loss_per_meter);
	ClassDB::bind_method(D_METHOD("get_high_frequency_loss_per_meter"),
			&VehicleAudioGuideData::get_high_frequency_loss_per_meter);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "high_frequency_loss_per_meter",
			PROPERTY_HINT_RANGE, "0,100,0.001,or_greater"),
			"set_high_frequency_loss_per_meter", "get_high_frequency_loss_per_meter");
}

void VehicleAudioTopologyData::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_nodes", "value"), &VehicleAudioTopologyData::set_nodes);
	ClassDB::bind_method(D_METHOD("get_nodes"), &VehicleAudioTopologyData::get_nodes);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "nodes", PROPERTY_HINT_ARRAY_TYPE,
			"VehicleAudioNodeData"), "set_nodes", "get_nodes");
	ClassDB::bind_method(D_METHOD("set_guides", "value"), &VehicleAudioTopologyData::set_guides);
	ClassDB::bind_method(D_METHOD("get_guides"), &VehicleAudioTopologyData::get_guides);
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "guides", PROPERTY_HINT_ARRAY_TYPE,
			"VehicleAudioGuideData"), "set_guides", "get_guides");
	ClassDB::bind_method(D_METHOD("set_source_names", "value"), &VehicleAudioTopologyData::set_source_names);
	ClassDB::bind_method(D_METHOD("get_source_names"), &VehicleAudioTopologyData::get_source_names);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "source_names"),
			"set_source_names", "get_source_names");
	ClassDB::bind_method(D_METHOD("set_source_nodes", "value"), &VehicleAudioTopologyData::set_source_nodes);
	ClassDB::bind_method(D_METHOD("get_source_nodes"), &VehicleAudioTopologyData::get_source_nodes);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "source_nodes"),
			"set_source_nodes", "get_source_nodes");
	ClassDB::bind_method(D_METHOD("set_source_gains", "value"), &VehicleAudioTopologyData::set_source_gains);
	ClassDB::bind_method(D_METHOD("get_source_gains"), &VehicleAudioTopologyData::get_source_gains);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "source_gains"),
			"set_source_gains", "get_source_gains");
	ClassDB::bind_method(D_METHOD("set_output_node", "value"), &VehicleAudioTopologyData::set_output_node);
	ClassDB::bind_method(D_METHOD("get_output_node"), &VehicleAudioTopologyData::get_output_node);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "output_node"), "set_output_node", "get_output_node");
	ClassDB::bind_method(D_METHOD("set_output_gain", "value"), &VehicleAudioTopologyData::set_output_gain);
	ClassDB::bind_method(D_METHOD("get_output_gain"), &VehicleAudioTopologyData::get_output_gain);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "output_gain", PROPERTY_HINT_RANGE,
			"0,10,0.001,or_greater"), "set_output_gain", "get_output_gain");
	ClassDB::bind_method(D_METHOD("set_output_tap_type", "value"),
			&VehicleAudioTopologyData::set_output_tap_type);
	ClassDB::bind_method(D_METHOD("get_output_tap_type"),
			&VehicleAudioTopologyData::get_output_tap_type);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "output_tap_type", PROPERTY_HINT_ENUM,
			"NodePressure,BoundaryRadiation"), "set_output_tap_type", "get_output_tap_type");
}

VehicleAudioFiringData::VehicleAudioFiringData() {
	configure_default_four_stroke();
}

void VehicleAudioFiringData::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_phases_degrees", "value"), &VehicleAudioFiringData::set_phases_degrees);
	ClassDB::bind_method(D_METHOD("get_phases_degrees"), &VehicleAudioFiringData::get_phases_degrees);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "phases_degrees"),
			"set_phases_degrees", "get_phases_degrees");
	ClassDB::bind_method(D_METHOD("set_source_names", "value"), &VehicleAudioFiringData::set_source_names);
	ClassDB::bind_method(D_METHOD("get_source_names"), &VehicleAudioFiringData::get_source_names);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "source_names"),
			"set_source_names", "get_source_names");
	ClassDB::bind_method(D_METHOD("set_gains", "value"), &VehicleAudioFiringData::set_gains);
	ClassDB::bind_method(D_METHOD("get_gains"), &VehicleAudioFiringData::get_gains);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "gains"),
			"set_gains", "get_gains");
	ClassDB::bind_method(D_METHOD("set_pulse_width_ms", "value"),
			&VehicleAudioFiringData::set_pulse_width_ms);
	ClassDB::bind_method(D_METHOD("get_pulse_width_ms"),
			&VehicleAudioFiringData::get_pulse_width_ms);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pulse_width_ms", PROPERTY_HINT_RANGE,
			"0.01,20,0.01,or_greater"), "set_pulse_width_ms", "get_pulse_width_ms");
	ClassDB::bind_method(D_METHOD("configure_default_four_stroke"),
			&VehicleAudioFiringData::configure_default_four_stroke);
}

void VehicleAudioFiringData::configure_default_four_stroke() {
	phases_degrees.resize(4);
	phases_degrees.set(0, 0.0f);
	phases_degrees.set(1, 180.0f);
	phases_degrees.set(2, 360.0f);
	phases_degrees.set(3, 540.0f);
	source_names.resize(4);
	source_names.set(0, "cylinder_1");
	source_names.set(1, "cylinder_3");
	source_names.set(2, "cylinder_4");
	source_names.set(3, "cylinder_2");
	gains.resize(4);
	gains.fill(1.0f);
}

} // namespace godot
