#include "turbo_data.h"

namespace godot {

void TurboData::_bind_methods() {
    // Spool up
    ClassDB::bind_method(D_METHOD("set_spool_up", "spool_up"), &TurboData::set_spool_up);
    ClassDB::bind_method(D_METHOD("get_spool_up"), &TurboData::get_spool_up);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spool_up", PROPERTY_HINT_RANGE, "0,5,0.01,or_greater"), "set_spool_up", "get_spool_up");

    // Spool down
    ClassDB::bind_method(D_METHOD("set_spool_down", "spool_down"), &TurboData::set_spool_down);
    ClassDB::bind_method(D_METHOD("get_spool_down"), &TurboData::get_spool_down);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spool_down", PROPERTY_HINT_RANGE, "0,5,0.01,or_greater"), "set_spool_down", "get_spool_down");

    // Max boost
    ClassDB::bind_method(D_METHOD("set_max_boost", "max_boost"), &TurboData::set_max_boost);
    ClassDB::bind_method(D_METHOD("get_max_boost"), &TurboData::get_max_boost);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_boost", PROPERTY_HINT_RANGE, "0,100,0.01,or_greater"), "set_max_boost", "get_max_boost");

    // Start RPM
    ClassDB::bind_method(D_METHOD("set_start_rpm", "start_rpm"), &TurboData::set_start_rpm);
    ClassDB::bind_method(D_METHOD("get_start_rpm"), &TurboData::get_start_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_rpm", PROPERTY_HINT_RANGE, "0,10000,1,or_greater"), "set_start_rpm", "get_start_rpm");

    // Max boost RPM
    ClassDB::bind_method(D_METHOD("set_max_boost_rpm", "max_boost_rpm"), &TurboData::set_max_boost_rpm);
    ClassDB::bind_method(D_METHOD("get_max_boost_rpm"), &TurboData::get_max_boost_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_boost_rpm", PROPERTY_HINT_RANGE, "0,15000,1,or_greater"), "set_max_boost_rpm", "get_max_boost_rpm");

    // Fall RPM
    ClassDB::bind_method(D_METHOD("set_fall_rpm", "fall_rpm"), &TurboData::set_fall_rpm);
    ClassDB::bind_method(D_METHOD("get_fall_rpm"), &TurboData::get_fall_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fall_rpm", PROPERTY_HINT_RANGE, "0,15000,1,or_greater"), "set_fall_rpm", "get_fall_rpm");
}

// Spool up
void TurboData::set_spool_up(real_t p_spool_up) {
    spool_up = p_spool_up;
}

real_t TurboData::get_spool_up() const {
    return spool_up;
}

// Spool down
void TurboData::set_spool_down(real_t p_spool_down) {
    spool_down = p_spool_down;
}

real_t TurboData::get_spool_down() const {
    return spool_down;
}

// Max boost
void TurboData::set_max_boost(real_t p_max_boost) {
    max_boost = p_max_boost;
}

real_t TurboData::get_max_boost() const {
    return max_boost;
}

// Start RPM
void TurboData::set_start_rpm(real_t p_start_rpm) {
    start_rpm = p_start_rpm;
}

real_t TurboData::get_start_rpm() const {
    return start_rpm;
}

// Max boost RPM
void TurboData::set_max_boost_rpm(real_t p_max_boost_rpm) {
    max_boost_rpm = p_max_boost_rpm;
}

real_t TurboData::get_max_boost_rpm() const {
    return max_boost_rpm;
}

// Fall RPM
void TurboData::set_fall_rpm(real_t p_fall_rpm) {
    fall_rpm = p_fall_rpm;
}

real_t TurboData::get_fall_rpm() const {
    return fall_rpm;
}

}