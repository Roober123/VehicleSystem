#pragma once

#include "SkidMarkBuffer.h"
#include "godot_cpp/classes/array_mesh.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/core/class_db.hpp"

namespace godot {

/// Grounded tire-solve output consumed by the skid visualizer.
struct SkidContactSample {
    Vector3 contact_position;
    Vector3 contact_normal;
    Vector3 tire_direction;
    Vector3 ground_velocity;
    real_t longitudinal_slip_velocity = real_t{0.0};
    real_t lateral_slip_velocity = real_t{0.0};
    real_t longitudinal_force = real_t{0.0};
    real_t lateral_force = real_t{0.0};
    real_t normal_load = real_t{0.0};
};

class TireSkid : public Node3D {
    GDCLASS(TireSkid, Node3D);

protected:
    static void _bind_methods();

public:
    TireSkid();
    ~TireSkid() override = default;

    void submit_sample(const SkidContactSample &sample, real_t dt);
    void stop_skid();

    void _ready() override;
    void _process(double delta) override;

    void set_ribbon_width(real_t width) { ribbon_width = width; }

    const SkidMarkBuffer &get_buffer() const { return buffer; }

private:
    static constexpr real_t INTENSITY_ONSET = real_t{2.0};
    static constexpr real_t FULL_INTENSITY = real_t{6.0};
    static constexpr real_t POINT_SPACING = real_t{0.10};
    static constexpr real_t RELEASE_GRACE = real_t{0.12};
    static constexpr real_t MAX_CONTINUOUS_DISPLACEMENT = real_t{2.0};
    static constexpr real_t STATIONARY_SPEED = real_t{0.10};
    static constexpr real_t STATIONARY_PATCH_LENGTH = real_t{0.04};
    static constexpr real_t GROUND_OFFSET = real_t{0.005};

    SkidMarkBuffer buffer;
    bool segment_active = false;
    Vector3 last_contact_position;
    real_t distance_since_section = real_t{0.0};
    real_t release_elapsed = real_t{0.0};

    MeshInstance3D *mesh_instance = nullptr;
    Ref<StandardMaterial3D> material;
    bool mesh_dirty = false;
    bool reparent_needed = true;
    int rebuild_skip = 0;
    real_t ribbon_width = real_t{0.15};

    real_t _calculate_intensity(const SkidContactSample &sample) const;
    SkidMarkSection _make_section(const SkidContactSample &sample,
                                  const Vector3 &world_center,
                                  real_t intensity,
                                  bool connected) const;
    void _start_segment(const SkidContactSample &sample, real_t intensity);
    void _append_moving_sections(const SkidContactSample &sample, real_t intensity);
    void _rebuild_mesh();
};

} // namespace godot
