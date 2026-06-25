#pragma once
#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/classes/curve.hpp"

namespace godot {

class VehicleEngineData : public Resource {
    GDCLASS(VehicleEngineData, Resource);
    Ref<Curve> torque_curve = nullptr;
    real_t idle_rpm = 850;
    real_t redline_rpm = 5000;
    real_t inertia = 1.0;
    real_t max_torque = 300; // Nm

    protected:
    static void _bind_methods();

    public:
    // Torque curve
    void set_torque_curve(const Ref<Curve> &p_curve);
    Ref<Curve> get_torque_curve() const;

    // Idle RPM
    void set_idle_rpm(real_t p_rpm);
    real_t get_idle_rpm() const;

    // Redline RPM
    void set_redline_rpm(real_t p_rpm);
    real_t get_redline_rpm() const;

    // Inertia
    void set_inertia(real_t p_inertia);
    real_t get_inertia() const;

    // Max torque
    void set_max_torque(real_t p_torque);
    real_t get_max_torque() const;
};


}