#pragma once
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/resource.hpp"
#include "godot_cpp/variant/packed_float64_array.hpp"

namespace godot {

class GearboxData : public Resource {
    GDCLASS(GearboxData, Resource);

    PackedFloat64Array gear_ratios = { 3.5, 2.1, 1.4, 1.0, 0.75 };
    real_t final_drive = 3.73;
    real_t clutch_max_torque = 400.0;
    real_t reverse_ratio = -3.0;    // Negative for backward motion

    real_t shift_time = 0.25;       
    real_t clutch_engage_speed = 10.0;
    real_t clutch_disengage_speed = 10.0;
    real_t upshift_rpm_ratio = 0.8;    
    real_t downshift_rpm_ratio = 0.25;  
    bool auto_mode = true;
    real_t driveshaft_drag = 0.05;  // Drivetrain rotational drag          

protected:
    static void _bind_methods();

public:
    // Gear ratios
    void set_gear_ratios(const PackedFloat64Array &p_ratios);
    PackedFloat64Array get_gear_ratios() const;

    // Final drive
    void set_final_drive(real_t p_value);
    real_t get_final_drive() const;

    // Clutch max torque
    void set_clutch_max_torque(real_t p_value);
    real_t get_clutch_max_torque() const;

    // Reverse ratio
    void set_reverse_ratio(real_t p_value);
    real_t get_reverse_ratio() const;

    void set_shift_time(real_t p_value);
    real_t get_shift_time() const;

    void set_clutch_engage_speed(real_t p_value);
    real_t get_clutch_engage_speed() const;

    void set_clutch_disengage_speed(real_t p_value);
    real_t get_clutch_disengage_speed() const;

    void set_upshift_rpm(real_t p_value);
    real_t get_upshift_rpm() const;

    void set_downshift_rpm(real_t p_value);
    real_t get_downshift_rpm() const;

    void set_auto_mode(bool p_value);
    bool get_auto_mode() const;

    void set_driveshaft_drag(real_t p_value);
    real_t get_driveshaft_drag() const;
};

} // namespace godot
