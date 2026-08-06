#pragma once

#include <array>
#include <cstddef>

#include "godot_cpp/variant/vector3.hpp"

namespace godot {

struct SkidMarkSection {
    Vector3 center;
    Vector3 normal;
    Vector3 lateral_direction;
    real_t intensity = real_t{0.0};
    bool connected_to_previous = false;
};

/// Fixed-capacity, oldest-first skid history. Pushing at capacity replaces the
/// oldest section and breaks the new oldest section's stale predecessor link.
class SkidMarkBuffer {
public:
    static constexpr std::size_t CAPACITY = 2048;

    std::size_t size() const { return count; }
    const SkidMarkSection &section_at(std::size_t ordered_index) const;

    void push(const SkidMarkSection &section);
    void darken_last_pair(real_t intensity);

private:
    std::array<SkidMarkSection, CAPACITY> sections{};
    std::size_t oldest = 0;
    std::size_t count = 0;
};

} // namespace godot
