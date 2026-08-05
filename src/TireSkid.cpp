#include "TireSkid.h"

#include <algorithm>
#include <cmath>

#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/surface_tool.hpp"

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
    material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);

    mesh_instance = memnew(MeshInstance3D);
    mesh_instance->set_name("SkidMesh");
    add_child(mesh_instance);
    mesh_instance->set_owner(this);

    Ref<ArrayMesh> empty_mesh;
    empty_mesh.instantiate();
    mesh_instance->set_mesh(empty_mesh);
}

void TireSkid::_process(double) {
    if (reparent_needed) {
        reparent_needed = false;
        Node *parent = get_parent();
        if (parent != nullptr && is_inside_tree()) {
            Node *root = get_tree()->get_current_scene();
            if (root != nullptr && root != parent)
                reparent(root, true);
        }
    }

    if (mesh_dirty) {
        rebuild_skip = (rebuild_skip + 1) % 3;
        if (rebuild_skip == 0) {
            _rebuild_mesh();
            mesh_dirty = false;
        }
    }
}

real_t TireSkid::_calculate_intensity(const SkidContactSample &sample) const {
    if (!(sample.normal_load > real_t{1e-6}) || !std::isfinite(sample.normal_load))
        return real_t{0.0};

    const real_t normalized_power =
            (std::abs(sample.longitudinal_force * sample.longitudinal_slip_velocity) +
             std::abs(sample.lateral_force * sample.lateral_slip_velocity)) /
            sample.normal_load;
    if (!std::isfinite(normalized_power))
        return real_t{0.0};

    return std::clamp((normalized_power - INTENSITY_ONSET) /
                              (FULL_INTENSITY - INTENSITY_ONSET),
                      real_t{0.0}, real_t{1.0});
}

SkidMarkSection TireSkid::_make_section(const SkidContactSample &sample,
                                        const Vector3 &world_center,
                                        real_t intensity,
                                        bool connected) const {
    const Vector3 raised_center = world_center + sample.contact_normal * GROUND_OFFSET;
    const auto local_point = [this](const Vector3 &point) {
        return is_inside_tree() ? to_local(point) : point;
    };
    SkidMarkSection section;
    section.center = local_point(raised_center);
    section.normal =
            (local_point(raised_center + sample.contact_normal) - section.center).normalized();
    const Vector3 world_lateral = sample.contact_normal.cross(sample.tire_direction).normalized();
    section.lateral_direction =
            (local_point(raised_center + world_lateral) - section.center).normalized();
    section.intensity = intensity;
    section.connected_to_previous = connected;
    return section;
}

void TireSkid::_start_segment(const SkidContactSample &sample, real_t intensity) {
    Vector3 patch_direction = sample.ground_velocity;
    if (patch_direction.length_squared() < STATIONARY_SPEED * STATIONARY_SPEED)
        patch_direction = sample.tire_direction;
    patch_direction.normalize();

    buffer.push(_make_section(sample,
                              sample.contact_position - patch_direction * STATIONARY_PATCH_LENGTH,
                              intensity, false));
    buffer.push(_make_section(sample, sample.contact_position, intensity, true));
    segment_active = true;
    last_contact_position = sample.contact_position;
    distance_since_section = real_t{0.0};
    release_elapsed = real_t{0.0};
    mesh_dirty = true;
}

void TireSkid::_append_moving_sections(const SkidContactSample &sample, real_t intensity) {
    const Vector3 displacement = sample.contact_position - last_contact_position;
    const real_t distance = displacement.length();
    if (distance <= real_t{0.0})
        return;

    const Vector3 direction = displacement / distance;
    Vector3 cursor = last_contact_position;
    real_t remaining = distance;
    while (distance_since_section + remaining >= POINT_SPACING) {
        const real_t step = POINT_SPACING - distance_since_section;
        cursor += direction * step;
        buffer.push(_make_section(sample, cursor, intensity, true));
        remaining -= step;
        distance_since_section = real_t{0.0};
        mesh_dirty = true;
    }
    distance_since_section += remaining;
    last_contact_position = sample.contact_position;
}

void TireSkid::submit_sample(const SkidContactSample &sample, real_t dt) {
    const real_t intensity = _calculate_intensity(sample);
    if (intensity <= real_t{0.0}) {
        if (!segment_active)
            return;
        release_elapsed += dt;
        if (release_elapsed >= RELEASE_GRACE)
            stop_skid();
        return;
    }

    release_elapsed = real_t{0.0};
    if (!segment_active) {
        _start_segment(sample, intensity);
        return;
    }

    if (last_contact_position.distance_to(sample.contact_position) >
        MAX_CONTINUOUS_DISPLACEMENT) {
        segment_active = false;
        _start_segment(sample, intensity);
        return;
    }

    if (sample.ground_velocity.length_squared() < STATIONARY_SPEED * STATIONARY_SPEED) {
        buffer.darken_last_pair(intensity);
        mesh_dirty = true;
        last_contact_position = sample.contact_position;
        return;
    }

    _append_moving_sections(sample, intensity);
}

void TireSkid::stop_skid() {
    segment_active = false;
    distance_since_section = real_t{0.0};
    release_elapsed = real_t{0.0};
}

void TireSkid::_rebuild_mesh() {
    if (mesh_instance == nullptr)
        return;

    Ref<SurfaceTool> surface;
    surface.instantiate();
    surface->begin(Mesh::PRIMITIVE_TRIANGLES);

    int vertex_offset = 0;
    for (std::size_t i = 1; i < buffer.size(); ++i) {
        const SkidMarkSection &previous = buffer.section_at(i - 1);
        const SkidMarkSection &current = buffer.section_at(i);
        if (!current.connected_to_previous)
            continue;

        const bool finite = previous.center.is_finite() && previous.normal.is_finite() &&
                            previous.lateral_direction.is_finite() &&
                            current.center.is_finite() && current.normal.is_finite() &&
                            current.lateral_direction.is_finite() &&
                            std::isfinite(previous.intensity) && std::isfinite(current.intensity);
        if (!finite)
            continue;

        const Vector3 previous_half = previous.lateral_direction * ribbon_width * real_t{0.5};
        const Vector3 current_half = current.lateral_direction * ribbon_width * real_t{0.5};
        const SkidMarkSection *ends[2] = {&previous, &current};
        const Vector3 halves[2] = {previous_half, current_half};
        for (int end = 0; end < 2; ++end) {
            const Color color(real_t{0.0}, real_t{0.0}, real_t{0.0},
                              ends[end]->intensity);
            surface->set_color(color);
            surface->set_normal(ends[end]->normal);
            surface->add_vertex(ends[end]->center - halves[end]);
            surface->set_color(color);
            surface->set_normal(ends[end]->normal);
            surface->add_vertex(ends[end]->center + halves[end]);
        }

        surface->add_index(vertex_offset + 0);
        surface->add_index(vertex_offset + 1);
        surface->add_index(vertex_offset + 2);
        surface->add_index(vertex_offset + 2);
        surface->add_index(vertex_offset + 1);
        surface->add_index(vertex_offset + 3);
        vertex_offset += 4;
    }

    Ref<ArrayMesh> rebuilt_mesh;
    rebuilt_mesh.instantiate();
    if (vertex_offset > 0) {
        surface->set_material(material);
        Ref<ArrayMesh> committed = surface->commit(rebuilt_mesh);
        if (committed.is_valid())
            rebuilt_mesh = committed;
    }
    mesh_instance->set_mesh(rebuilt_mesh);
}

} // namespace godot
