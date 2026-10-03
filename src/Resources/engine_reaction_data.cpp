#include "engine_reaction_data.h"

#include <algorithm>
#include <cmath>

namespace godot {

void EngineReactionData::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_enabled", "value"), &EngineReactionData::set_enabled);
    ClassDB::bind_method(D_METHOD("get_enabled"), &EngineReactionData::get_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "get_enabled");

    ADD_GROUP("Torque Reaction", "");
    ClassDB::bind_method(D_METHOD("set_reaction_strength", "value"), &EngineReactionData::set_reaction_strength);
    ClassDB::bind_method(D_METHOD("get_reaction_strength"), &EngineReactionData::get_reaction_strength);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reaction_strength", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_reaction_strength", "get_reaction_strength");
    ClassDB::bind_method(D_METHOD("set_maximum_torque", "value"), &EngineReactionData::set_maximum_torque);
    ClassDB::bind_method(D_METHOD("get_maximum_torque"), &EngineReactionData::get_maximum_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "maximum_torque", PROPERTY_HINT_RANGE, "0,2000,1,or_greater,suffix:Nm"), "set_maximum_torque", "get_maximum_torque");

    ADD_GROUP("Vibration", "");
    ClassDB::bind_method(D_METHOD("set_vibration_strength", "value"), &EngineReactionData::set_vibration_strength);
    ClassDB::bind_method(D_METHOD("get_vibration_strength"), &EngineReactionData::get_vibration_strength);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vibration_strength", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_vibration_strength", "get_vibration_strength");
    ClassDB::bind_method(D_METHOD("set_idle_vibration_max_rpm", "value"), &EngineReactionData::set_idle_vibration_max_rpm);
    ClassDB::bind_method(D_METHOD("get_idle_vibration_max_rpm"), &EngineReactionData::get_idle_vibration_max_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "idle_vibration_max_rpm", PROPERTY_HINT_RANGE, "0,3000,10,or_greater,suffix:RPM"), "set_idle_vibration_max_rpm", "get_idle_vibration_max_rpm");
    ClassDB::bind_method(D_METHOD("set_idle_vibration_multiplier", "value"), &EngineReactionData::set_idle_vibration_multiplier);
    ClassDB::bind_method(D_METHOD("get_idle_vibration_multiplier"), &EngineReactionData::get_idle_vibration_multiplier);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "idle_vibration_multiplier", PROPERTY_HINT_RANGE, "0,5,0.05,or_greater"), "set_idle_vibration_multiplier", "get_idle_vibration_multiplier");
    ClassDB::bind_method(D_METHOD("set_idle_vibration_frequency", "value"), &EngineReactionData::set_idle_vibration_frequency);
    ClassDB::bind_method(D_METHOD("get_idle_vibration_frequency"), &EngineReactionData::get_idle_vibration_frequency);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "idle_vibration_frequency", PROPERTY_HINT_RANGE, "0,30,0.1,or_greater,suffix:Hz"), "set_idle_vibration_frequency", "get_idle_vibration_frequency");
    ClassDB::bind_method(D_METHOD("set_redline_vibration_frequency", "value"), &EngineReactionData::set_redline_vibration_frequency);
    ClassDB::bind_method(D_METHOD("get_redline_vibration_frequency"), &EngineReactionData::get_redline_vibration_frequency);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "redline_vibration_frequency", PROPERTY_HINT_RANGE, "0,30,0.1,or_greater,suffix:Hz"), "set_redline_vibration_frequency", "get_redline_vibration_frequency");

    ADD_GROUP("Speed Fade", "");
    ClassDB::bind_method(D_METHOD("set_speed_fade_start_kph", "value"), &EngineReactionData::set_speed_fade_start_kph);
    ClassDB::bind_method(D_METHOD("get_speed_fade_start_kph"), &EngineReactionData::get_speed_fade_start_kph);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed_fade_start_kph", PROPERTY_HINT_RANGE, "0,100,1,or_greater,suffix:km/h"), "set_speed_fade_start_kph", "get_speed_fade_start_kph");
    ClassDB::bind_method(D_METHOD("set_speed_fade_end_kph", "value"), &EngineReactionData::set_speed_fade_end_kph);
    ClassDB::bind_method(D_METHOD("get_speed_fade_end_kph"), &EngineReactionData::get_speed_fade_end_kph);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed_fade_end_kph", PROPERTY_HINT_RANGE, "1,150,1,or_greater,suffix:km/h"), "set_speed_fade_end_kph", "get_speed_fade_end_kph");
}

void EngineReactionData::set_reaction_strength(real_t value) {
    if (std::isfinite(value)) reaction_strength = std::max(value, real_t{0.0});
}
void EngineReactionData::set_vibration_strength(real_t value) {
    if (std::isfinite(value)) vibration_strength = std::max(value, real_t{0.0});
}
void EngineReactionData::set_idle_vibration_max_rpm(real_t value) {
    if (std::isfinite(value)) idle_vibration_max_rpm = std::max(value, real_t{0.0});
}
void EngineReactionData::set_idle_vibration_multiplier(real_t value) {
    if (std::isfinite(value)) idle_vibration_multiplier = std::max(value, real_t{0.0});
}
void EngineReactionData::set_idle_vibration_frequency(real_t value) {
    if (std::isfinite(value)) idle_vibration_frequency = std::max(value, real_t{0.0});
}
void EngineReactionData::set_redline_vibration_frequency(real_t value) {
    if (std::isfinite(value)) redline_vibration_frequency = std::max(value, real_t{0.0});
}
void EngineReactionData::set_maximum_torque(real_t value) {
    if (std::isfinite(value)) maximum_torque = std::max(value, real_t{0.0});
}
void EngineReactionData::set_speed_fade_start_kph(real_t value) {
    if (std::isfinite(value)) speed_fade_start_kph = std::max(value, real_t{0.0});
}
void EngineReactionData::set_speed_fade_end_kph(real_t value) {
    if (std::isfinite(value)) speed_fade_end_kph = std::max(value, real_t{1.0});
}

} // namespace godot
