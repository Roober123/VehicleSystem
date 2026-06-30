#pragma once

#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/resource.hpp"

namespace godot {

class VehicleAerodynamicsData : public Resource {
    GDCLASS(VehicleAerodynamicsData, Resource);

protected:
    static void _bind_methods();

public:
    VehicleAerodynamicsData() = default;
    ~VehicleAerodynamicsData() override = default;

    void set_drag_coefficient(real_t v);
    real_t get_drag_coefficient() const;

    void set_downforce_coefficient(real_t v);
    real_t get_downforce_coefficient() const;

    void set_yaw_damping_coefficient(real_t v);
    real_t get_yaw_damping_coefficient() const;

    real_t drag_coefficient = 0.35;
    real_t downforce_coefficient = 0.15;
    real_t yaw_damping_coefficient = 2.0;
};

}