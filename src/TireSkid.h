#pragma once

#include "godot_cpp/classes/node3d.hpp"
#include "godot_cpp/classes/mesh_instance3d.hpp"
#include "godot_cpp/classes/array_mesh.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/classes/surface_tool.hpp"
#include "godot_cpp/core/class_db.hpp"
#include "godot_cpp/variant/color.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace godot {

struct SkidPoint { // should be stored in world space
    Vector3 left;
    Vector3 right;
    Vector3 center;
    Vector3 normal;
};

struct SkidRibbon {
    std::array<SkidPoint, 128> points;
    int point_count = 0;
    bool active = false;
    real_t age = real_t{0.0};
    real_t travel_distance = real_t{0.0};  // accumulated distance since last point
};

class TireSkid : public Node3D {
    GDCLASS(TireSkid, Node3D);

protected:
    static void _bind_methods();

public:
    TireSkid();
    ~TireSkid() override = default;

    // Called every physics tick from the wheel
    void update_skid(const Vector3 &contact_position,
                     const Vector3 &contact_normal,
                     const Vector3 &travel_direction);
    void stop_skid();

    void _ready() override;
    void _process(double delta) override;

    void set_ribbon_width(real_t w) { ribbon_width = w; }

private:

    static constexpr int MAX_POINTS = 128;
    static constexpr int MAX_RIBBONS = 16;
    static constexpr real_t POINT_SPACING = real_t{0.3};
    static constexpr real_t GROUND_OFFSET = real_t{0.005};
    static constexpr real_t LIFETIME = real_t{20.0};


    std::array<SkidRibbon, MAX_RIBBONS> ribbons;
    int active_ribbon_index = -1;

    // Mesh
    MeshInstance3D *mesh_instance = nullptr;
    Ref<ArrayMesh> mesh;
    Ref<StandardMaterial3D> material;
    bool mesh_dirty = false;
    bool reparent_needed = true;
    int rebuild_skip = 0;  // throttle: only rebuild every 3rd _process call

    real_t ribbon_width = real_t{0.15};
    // Internal helpers
    int _find_or_create_active_ribbon();
    void _finish_active_ribbon();
    void _rebuild_mesh();
    SkidPoint _get_skidpoint_from_data(const Vector3 &contact_position, const Vector3 &contact_normal,
                    const Vector3 &travel_direction);
};

}
