#pragma once
#include "RotationalBody.h"
#include "godot_cpp/classes/curve.hpp"
#include "Resources/vehicle_engine_data.h"
#include "Drivetrain/Turbo.h"


namespace godot {

class VehicleEngine : public RotationalBody {
    Ref<Curve> torque_curve = nullptr;
    real_t idle_rpm = 750.0;
    real_t redline_rpm = 4500.0;
    real_t max_torque = 300.0;
    real_t engine_braking = 0.5;

    bool rev_limit_cut = false;
    real_t effective_drive_torque = 0.0;
    real_t generated_torque = 0.0;
    real_t self_torque = 0.0;

    Turbo* turbo = nullptr;
    

    public:
    VehicleEngine(const Ref<VehicleEngineData>& r) {
        if (r == nullptr) {
            return;
        }
        torque_curve = r->get_torque_curve();
        idle_rpm = r->get_idle_rpm();
        redline_rpm = r->get_redline_rpm();
        max_torque = r->get_max_torque();
        inertia = r->get_inertia();
        drag = r->get_engine_drag();
        engine_braking = r->get_engine_braking();
        angular_velocity = idle_rpm / 60 *  2 * Math_PI;
    }
    VehicleEngine() {}
    
    void set_turbo(Turbo *t) { turbo = t; }

    real_t get_rpm_normalized() const;
    real_t get_torque() const;
    // Generated torque includes idle support; self torque also subtracts
    // braking/drag. Both are cached before clutch coupling changes the body.
    real_t get_generated_torque() const { return generated_torque; }
    real_t get_self_torque() const { return self_torque; }
    real_t get_idle_rpm() const { return idle_rpm; }
    real_t get_redline_rpm() const { return redline_rpm; }
    real_t get_turbo_boost() const;
    real_t get_rpm() const { return angular_velocity * 60.0 / 2.0 / Math_PI; }

    void accumulate_torque(real_t dt);
    void integrate(real_t dt);

    real_t throttle = 0.0;
    
    
    

};



}
