#include "turbo_data.h"

namespace godot {

void TurboData::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_max_boost_bar", "value"),
                         &TurboData::set_max_boost_bar);
    ClassDB::bind_method(D_METHOD("get_max_boost_bar"),
                         &TurboData::get_max_boost_bar);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_boost_bar",
                              PROPERTY_HINT_RANGE, "0,20,0.01,or_greater"),
                 "set_max_boost_bar", "get_max_boost_bar");

    ClassDB::bind_method(D_METHOD("set_full_boost_rpm", "value"),
                         &TurboData::set_full_boost_rpm);
    ClassDB::bind_method(D_METHOD("get_full_boost_rpm"),
                         &TurboData::get_full_boost_rpm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "full_boost_rpm",
                              PROPERTY_HINT_RANGE, "1,100000,1,or_greater"),
                 "set_full_boost_rpm", "get_full_boost_rpm");

    ClassDB::bind_method(D_METHOD("set_lag_seconds", "value"),
                         &TurboData::set_lag_seconds);
    ClassDB::bind_method(D_METHOD("get_lag_seconds"),
                         &TurboData::get_lag_seconds);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lag_seconds",
                              PROPERTY_HINT_RANGE, "0.001,60,0.001,or_greater"),
                 "set_lag_seconds", "get_lag_seconds");
}

void TurboData::set_max_boost_bar(real_t value) {
    max_boost_bar = value;
}

real_t TurboData::get_max_boost_bar() const {
    return max_boost_bar;
}

void TurboData::set_full_boost_rpm(real_t value) {
    full_boost_rpm = value;
}

real_t TurboData::get_full_boost_rpm() const {
    return full_boost_rpm;
}

void TurboData::set_lag_seconds(real_t value) {
    lag_seconds = value;
}

real_t TurboData::get_lag_seconds() const {
    return lag_seconds;
}

} // namespace godot
