#include "TireSkid.h"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/scene_tree.hpp"

#include <cmath>

namespace godot {

void TireSkid::_bind_methods() {
    
}

TireSkid::TireSkid() {
    set_process(true);
}


void TireSkid::_ready() {
    if (Engine::get_singleton()->is_editor_hint())
        return;

    reparent_needed = true;

    material.instantiate();
    material->set_albedo(Color(real_t{0.05}, real_t{0.05}, real_t{0.05}, real_t{1.0}));
    material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
    material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);

    // Create mesh instance
    mesh_instance = memnew(MeshInstance3D);
    mesh_instance->set_name("SkidMesh");
    add_child(mesh_instance);
    mesh_instance->set_owner(this);

    mesh.instantiate();
    mesh_instance->set_mesh(mesh);
}

void TireSkid::_process(double delta) {
    if (reparent_needed) {
        reparent_needed = false;
        Node *parent = get_parent();
        if (parent && is_inside_tree()) {
            Node *root = get_tree()->get_current_scene();
            if (root && root != parent) {
                reparent(root, true);
            }
        }
    }

    if (mesh_dirty) {
        rebuild_skip = (rebuild_skip + 1) % 3;
        if (rebuild_skip == 0) {
            _rebuild_mesh();
            mesh_dirty = false;
        }
    }
    for (int i = 0; i < MAX_RIBBONS; i++)
        if (!ribbons[i].active)
            ribbons[i].age += delta;
}

void TireSkid::stop_skid() {
    if (active_ribbon_index < 0) return;
    if (ribbons[active_ribbon_index].active == true)
        _finish_active_ribbon();
}

void TireSkid::update_skid(const Vector3 &contact_position,
                            const Vector3 &contact_normal,
                            const Vector3 &travel_direction) {
    if (active_ribbon_index < 0) {
        // First point of a new skid event — create a ribbon
        int idx = _find_or_create_active_ribbon();
        if (idx < 0) return;
        SkidRibbon &rib = ribbons[idx];
        rib.travel_distance = real_t{0.0};
        rib.points[rib.point_count++] = _get_skidpoint_from_data(
            contact_position, contact_normal, travel_direction);
        mesh_dirty = true;
        return;
    }
    if (ribbons[active_ribbon_index].point_count == 0)
        return;

    SkidRibbon &rib = ribbons[active_ribbon_index];
    SkidPoint &last = rib.points[rib.point_count - 1];
    real_t dist = std::sqrt(last.center.distance_squared_to(contact_position));
    rib.travel_distance += dist;

    if (rib.travel_distance < POINT_SPACING)
        return;

    // Reset accumulator (keep the remainder to avoid drift)
    rib.travel_distance -= POINT_SPACING;

    int idx = _find_or_create_active_ribbon();
    ribbons[idx].points[ribbons[idx].point_count++] = _get_skidpoint_from_data(
        contact_position, contact_normal, travel_direction);
    mesh_dirty = true;
}

int TireSkid::_find_or_create_active_ribbon() {
    if (active_ribbon_index != -1 && 
        ribbons[active_ribbon_index].point_count < MAX_POINTS &&
        ribbons[active_ribbon_index].active)
            return active_ribbon_index;
    if (active_ribbon_index >= 0 && ribbons[active_ribbon_index].point_count == MAX_POINTS)
        _finish_active_ribbon();

    int oldest_idx = -1;
    for (int i = 0; i < MAX_RIBBONS; i++)
        if (ribbons[i].active == false 
            && (oldest_idx == -1 || ribbons[i].age > ribbons[oldest_idx].age))
            oldest_idx = i;

    if (oldest_idx < 0) {
        return -1;
    }

    ribbons[oldest_idx].active = true;
    ribbons[oldest_idx].point_count = 0;
    ribbons[oldest_idx].age = 0.0;
    ribbons[oldest_idx].travel_distance = real_t{0.0};
    mesh_dirty = true;
    active_ribbon_index = oldest_idx;
    return oldest_idx;
}

void TireSkid::_finish_active_ribbon() {
    if (active_ribbon_index < 0) return;
    ribbons[active_ribbon_index].active = false;
    ribbons[active_ribbon_index].age = 0.0;
    active_ribbon_index = -1;
}

SkidPoint TireSkid::_get_skidpoint_from_data(const Vector3 &contact_position, const Vector3 &contact_normal,
                    const Vector3 &travel_direction) {
    SkidPoint s;
    Vector3 pos = contact_position + contact_normal * GROUND_OFFSET;
    s.normal = to_local(pos + contact_normal) - to_local(pos);
    s.normal.normalize();
    s.center = to_local(pos);
    Vector3 r_vec = contact_normal.cross(travel_direction);
    if (r_vec.length_squared() < 1e-6f) r_vec = Vector3(1.0, 0.0, 0.0);
    r_vec.normalize();
    s.left = to_local(pos - r_vec * ribbon_width * real_t{0.5});
    s.right = to_local(pos + r_vec * ribbon_width * real_t{0.5});
    return s;
}



void TireSkid::_rebuild_mesh() {
    int total_points = 0;
    for (int r = 0; r < MAX_RIBBONS; ++r) {
        if (ribbons[r].point_count > 0)
            total_points += ribbons[r].point_count;
    }

    if (total_points < 2) {
        // Assign a fresh empty mesh to clear old geometry
        mesh.instantiate();
        mesh_instance->set_mesh(mesh);
        return;
    }

    Ref<SurfaceTool> st;
    st.instantiate();
    st->begin(Mesh::PRIMITIVE_TRIANGLES);

    int vertex_offset = 0;  // tracks global vertex index across ribbons

    for (int r = 0; r < MAX_RIBBONS; ++r) {
        const SkidRibbon &rib = ribbons[r];
        if (rib.point_count < 2)
            continue;

        for (int i = 0; i < rib.point_count; ++i) {
            const SkidPoint &pt = rib.points[i];

            st->set_color(Color(real_t{0.0}, real_t{0.0}, real_t{0.0}, 1.0));
            st->set_normal(pt.normal);
            st->add_vertex(pt.left);

            st->set_color(Color(real_t{0.0}, real_t{0.0}, real_t{0.0}, 1.0));
            st->set_normal(pt.normal);
            st->add_vertex(pt.right);
        }

        // Build triangle strip: global vertices are at
        // [vertex_offset, vertex_offset+1, vertex_offset+2, ...]
        // Each point i contributes: L_i at (vertex_offset + i*2), R_i at (vertex_offset + i*2 + 1)
        for (int i = 0; i < rib.point_count - 1; ++i) {
            int base = vertex_offset + i * 2;

            // Tri 1: L_i  R_i  L_{i+1}
            st->add_index(base + 0);
            st->add_index(base + 1);
            st->add_index(base + 2);

            // Tri 2: L_{i+1}  R_i  R_{i+1}
            st->add_index(base + 2);
            st->add_index(base + 1);
            st->add_index(base + 3);
        }

        vertex_offset += rib.point_count * 2;
    }

    st->set_material(material);
    mesh->clear_surfaces();
    Ref<ArrayMesh> new_mesh = st->commit(mesh);
    if (new_mesh.is_valid()) {
        mesh = new_mesh;
        mesh_instance->set_mesh(mesh);
    }
}

} // namespace godot
