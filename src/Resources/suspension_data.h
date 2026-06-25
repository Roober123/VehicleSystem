#pragma once
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/classes/resource.hpp"

namespace godot {

class SuspensionData : public Resource {
    GDCLASS(SuspensionData, Resource);

    protected:
    static void _bind_methods();

    public:
    SuspensionData() = default;
    ~SuspensionData() override = default;

    void set_suspension_length(real_t value);
    real_t get_suspension_length();

    void set_rest_compression(real_t value);
    real_t get_rest_compression();

    void set_damping_ratio(real_t value);
    real_t get_damping_ratio();


    real_t suspension_length = 1.0;
    real_t rest_compression = 0.25;
    real_t damping_ratio = 0.5;
    
};

} // namespace godot