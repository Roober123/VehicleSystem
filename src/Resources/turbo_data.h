#pragma once

#include "godot_cpp/classes/resource.hpp"

namespace godot {

/// Authoring data for the minimal load-driven turbo model. Values are authored
/// within the Inspector ranges and copied into Turbo during configuration.
class TurboData : public Resource {
    GDCLASS(TurboData, Resource);

    real_t max_boost_bar = real_t{1.0};
    real_t full_boost_rpm = real_t{3000.0};
    real_t lag_seconds = real_t{0.6};

protected:
    static void _bind_methods();

public:
    void set_max_boost_bar(real_t value);
    real_t get_max_boost_bar() const;

    void set_full_boost_rpm(real_t value);
    real_t get_full_boost_rpm() const;

    void set_lag_seconds(real_t value);
    real_t get_lag_seconds() const;
};

} // namespace godot
