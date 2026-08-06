#include "SkidMarkBuffer.h"

#include <algorithm>

namespace godot {

const SkidMarkSection &SkidMarkBuffer::section_at(std::size_t ordered_index) const {
    return sections[(oldest + ordered_index) % CAPACITY];
}

void SkidMarkBuffer::push(const SkidMarkSection &section) {
    if (count < CAPACITY) {
        sections[(oldest + count) % CAPACITY] = section;
        ++count;
        return;
    }

    sections[oldest] = section;
    oldest = (oldest + 1) % CAPACITY;
    sections[oldest].connected_to_previous = false;
}

void SkidMarkBuffer::darken_last_pair(real_t intensity) {
    if (count < 2)
        return;

    const real_t amount = std::clamp(intensity, real_t{0.0}, real_t{1.0});
    for (std::size_t i = count - 2; i < count; ++i) {
        SkidMarkSection &section = sections[(oldest + i) % CAPACITY];
        section.intensity += (real_t{1.0} - section.intensity) * amount;
    }
}

} // namespace godot
