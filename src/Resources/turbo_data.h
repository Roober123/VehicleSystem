#pragma once
#include "godot_cpp/classes/resource.hpp"

namespace godot {

class TurboData : public Resource {
    GDCLASS(TurboData, Resource);

    real_t spool_up = 1.5;
    real_t spool_down = 2.5;
    real_t max_boost = 1; 
    real_t start_rpm = 2000;
    real_t max_boost_rpm = 2800;
    real_t fall_rpm = 6000;

protected:
    static void _bind_methods();

public:
    void set_spool_up(real_t p_spool_up);
    real_t get_spool_up() const;

    void set_spool_down(real_t p_spool_down);
    real_t get_spool_down() const;

    void set_max_boost(real_t p_max_boost);
    real_t get_max_boost() const;

    void set_start_rpm(real_t p_start_rpm);
    real_t get_start_rpm() const;

    void set_max_boost_rpm(real_t p_max_boost_rpm);
    real_t get_max_boost_rpm() const;

    void set_fall_rpm(real_t p_fall_rpm);
    real_t get_fall_rpm() const;
};

}