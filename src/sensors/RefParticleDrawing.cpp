#include "RefParticleDrawing.hpp"

#include <cmath>

namespace {
constexpr uint32_t DEBUG_FRAME_INTERVAL_MS = 50;
constexpr uint32_t DEBUG_ORBIT_PERIOD_MS = 4000;
constexpr float DEBUG_ORBIT_RADIANS = 6.28318530718f;

static_assert(DEBUG_FRAME_INTERVAL_MS * 1000 >= REF_MAX_PACKET_DELAY, "Debug frames must respect the referee packet interval");
}

RefParticleDrawing::RefParticleDrawing(RefDrawing &draw) : draw(draw) {}

bool RefParticleDrawing::make_particle(const char *name, uint16_t center_x, uint16_t center_y, uint8_t layer, ClientGraphicColor color) {
    return draw.draw_circle(name, center_x, center_y, PARTICLE_RADIUS, PARTICLE_WIDTH, layer, color);
}

bool RefParticleDrawing::debug_mode() {
    const uint32_t now_ms = millis();
    if (debug_attempted) {
        // Unsigned subtraction keeps the interval correct across millis() rollover.
        const uint32_t elapsed_ms = now_ms - debug_last_attempt_ms;
        if (elapsed_ms < DEBUG_FRAME_INTERVAL_MS) {
            return false;
        }
        debug_phase_ms = (debug_phase_ms + elapsed_ms % DEBUG_ORBIT_PERIOD_MS) % DEBUG_ORBIT_PERIOD_MS;
    }

    // Throttle failed attempts too, so an unavailable connection cannot flood the loop.
    debug_attempted = true;
    debug_last_attempt_ms = now_ms;

    const float angle = DEBUG_ORBIT_RADIANS * debug_phase_ms / DEBUG_ORBIT_PERIOD_MS;
    const uint16_t center_x = static_cast<uint16_t>(960.0f + 240.0f * std::cos(angle) + 0.5f);
    const uint16_t center_y = static_cast<uint16_t>(540.0f + 120.0f * std::sin(angle) + 0.5f);
    const ClientGraphicOperation operation = debug_added ? ClientGraphicOperation::EDIT : ClientGraphicOperation::ADD;
    const bool accepted = draw.draw_circle("pdb", center_x, center_y, PARTICLE_RADIUS, PARTICLE_WIDTH, PARTICLE_LAYER, PARTICLE_COLOR, operation);
    if (accepted) {
        debug_added = true;
    }
    return accepted;
}
