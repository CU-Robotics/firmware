#include "status_lights.hpp"

void StatusLights::set(Slot slot, Color color) {
    constexpr uint16_t slot_count = static_cast<uint16_t>(Slot::Count);
    uint16_t pixels_per_slot = strip_.num_pixels() / slot_count;

    if (pixels_per_slot == 0) {
        pixels_per_slot = 1;
    }

    uint16_t first = static_cast<uint16_t>(slot) * pixels_per_slot;
    strip_.set_range(first, pixels_per_slot, color.r, color.g, color.b);
}
