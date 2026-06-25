#pragma once
#include "RotationalBody.h"
#include "godot_cpp/classes/curve.hpp"
#include "Resources/vehicle_engine_data.h"


namespace godot {

class VehicleEngine : public RotationalBody {
    Ref<Curve> torque_curve = nullptr;
    real_t idle_rpm = 750.0;
    real_t redline_rpm = 4500.0;
    real_t max_torque = 300.0;
    

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
        
    }
    VehicleEngine() {}
    
    real_t get_rpm_normalized();
    real_t get_torque();
    real_t get_available_torque();
    

    void accumulate_torque();
    void integrate(real_t dt);

    real_t throttle = 0.0;
    
    
    

};



}