#include "gearbox_data.h"

namespace godot {

void GearboxData::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_gear_ratios", "ratios"), &GearboxData::set_gear_ratios);
    ClassDB::bind_method(D_METHOD("get_gear_ratios"), &GearboxData::get_gear_ratios);
    ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT64_ARRAY, "gear_ratios"), "set_gear_ratios", "get_gear_ratios");

    ClassDB::bind_method(D_METHOD("set_final_drive", "value"), &GearboxData::set_final_drive);
    ClassDB::bind_method(D_METHOD("get_final_drive"), &GearboxData::get_final_drive);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "final_drive", PROPERTY_HINT_RANGE, "0.1,20.0,0.01"), "set_final_drive", "get_final_drive");

    ClassDB::bind_method(D_METHOD("set_clutch_max_torque", "value"), &GearboxData::set_clutch_max_torque);
    ClassDB::bind_method(D_METHOD("get_clutch_max_torque"), &GearboxData::get_clutch_max_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "clutch_max_torque", PROPERTY_HINT_RANGE, "0,10000,1,or_greater"), "set_clutch_max_torque", "get_clutch_max_torque");

    // Reverse ratio
    ClassDB::bind_method(D_METHOD("set_reverse_ratio", "value"), &GearboxData::set_reverse_ratio);
    ClassDB::bind_method(D_METHOD("get_reverse_ratio"), &GearboxData::get_reverse_ratio);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reverse_ratio", PROPERTY_HINT_RANGE, "-20.0,-0.1,0.01"), "set_reverse_ratio", "get_reverse_ratio");

    // Shift time
    ClassDB::bind_method(D_METHOD("set_shift_time", "value"), &GearboxData::set_shift_time);
    ClassDB::bind_method(D_METHOD("get_shift_time"), &GearboxData::get_shift_time);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shift_time", PROPERTY_HINT_RANGE, "0.05,2.0,0.01"), "set_shift_time", "get_shift_time");

    // Upshift RPM ratio (normalized 0-1, fraction of RPM range)
    ClassDB::bind_method(D_METHOD("set_upshift_rpm", "value"), &GearboxData::set_upshift_rpm);
    ClassDB::bind_method(D_METHOD("get_upshift_rpm"), &GearboxData::get_upshift_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "upshift_rpm", PROPERTY_HINT_RANGE, "0.0,1.0,0.01"), "set_upshift_rpm", "get_upshift_rpm");

    // Downshift RPM ratio (normalized 0-1, fraction of RPM range)
    ClassDB::bind_method(D_METHOD("set_downshift_rpm", "value"), &GearboxData::set_downshift_rpm);
    ClassDB::bind_method(D_METHOD("get_downshift_rpm"), &GearboxData::get_downshift_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "downshift_rpm", PROPERTY_HINT_RANGE, "0.0,1.0,0.01"), "set_downshift_rpm", "get_downshift_rpm");

    // Auto mode
    ClassDB::bind_method(D_METHOD("set_auto_mode", "value"), &GearboxData::set_auto_mode);
    ClassDB::bind_method(D_METHOD("get_auto_mode"), &GearboxData::get_auto_mode);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_mode"), "set_auto_mode", "get_auto_mode");
}

void GearboxData::set_gear_ratios(const PackedFloat64Array &p_ratios) {
    gear_ratios = p_ratios;
}

PackedFloat64Array GearboxData::get_gear_ratios() const {
    return gear_ratios;
}

void GearboxData::set_final_drive(real_t p_value) {
    final_drive = p_value;
}

real_t GearboxData::get_final_drive() const {
    return final_drive;
}

void GearboxData::set_clutch_max_torque(real_t p_value) {
    clutch_max_torque = p_value;
}

real_t GearboxData::get_clutch_max_torque() const {
    return clutch_max_torque;
}

void GearboxData::set_reverse_ratio(real_t p_value) {
    reverse_ratio = p_value;
}

real_t GearboxData::get_reverse_ratio() const {
    return reverse_ratio;
}

void GearboxData::set_shift_time(real_t p_value) {
    shift_time = p_value;
}

real_t GearboxData::get_shift_time() const {
    return shift_time;
}

void GearboxData::set_upshift_rpm(real_t p_value) {
    upshift_rpm_ratio = p_value;
}

real_t GearboxData::get_upshift_rpm() const {
    return upshift_rpm_ratio;
}

void GearboxData::set_downshift_rpm(real_t p_value) {
    downshift_rpm_ratio = p_value;
}

real_t GearboxData::get_downshift_rpm() const {
    return downshift_rpm_ratio;
}

void GearboxData::set_auto_mode(bool p_value) {
    auto_mode = p_value;
}

bool GearboxData::get_auto_mode() const {
    return auto_mode;
}

} // namespace godot
