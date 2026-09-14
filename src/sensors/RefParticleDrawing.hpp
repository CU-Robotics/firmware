#pragma once

#include "RefDrawing.hpp"

/// @brief Particle circle radius in pixels.
constexpr uint16_t PARTICLE_RADIUS = 6;
/// @brief Particle circle line width in pixels.
constexpr uint16_t PARTICLE_WIDTH = 3;
/// @brief Default drawing layer shared by particles and the debug demo.
constexpr uint8_t PARTICLE_LAYER = 9;
/// @brief Default drawing color shared by particles and the debug demo.
constexpr ClientGraphicColor PARTICLE_COLOR = ClientGraphicColor::CYAN;

/// @brief Helper for drawing particles on the Player's Client.
class RefParticleDrawing {
  public:
    /// @brief Construct a particle drawing helper.
    /// @param draw Drawing helper, which must outlive this object.
    explicit RefParticleDrawing(RefDrawing &draw);

    /// @brief Add a particle circle, defaulting to layer 9 and cyan.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param center_x Particle center x-coordinate in pixels.
    /// @param center_y Particle center y-coordinate in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @return true when the drawing packet is accepted for transmission.
    bool make_particle(const char *name, uint16_t center_x, uint16_t center_y, uint8_t layer = PARTICLE_LAYER, ClientGraphicColor color = PARTICLE_COLOR);

    /// @brief Animate a cyan particle around (960, 540) on a 1920x1080 display.
    /// @note Call repeatedly from the main loop on the same instance. Never blocks;
    /// attempts at most one packet every 50 ms, including retries after rejection.
    /// @note Reserves figure name "pdb" on layer 9. Uses a 6-pixel radius and
    /// 3-pixel stroke, orbiting an oval with semi-axes 240 and 120 pixels every 4 s.
    /// @note Sends ADD until accepted, then EDIT. Recreate this helper if the
    /// client drawings are cleared or the client reconnects.
    /// @return true when a drawing packet is accepted; false when throttled or rejected.
    bool debug_mode();

  private:
    /// @brief Drawing helper used to send particle graphics.
    RefDrawing &draw;
    /// @brief Whether the debug particle's ADD packet has been accepted.
    bool debug_added = false;
    /// @brief Whether a debug transmission has been attempted, including at time zero.
    bool debug_attempted = false;
    /// @brief Time of the last debug transmission attempt, in milliseconds.
    uint32_t debug_last_attempt_ms = 0;
    /// @brief Elapsed animation time within the current orbit, in milliseconds.
    uint32_t debug_phase_ms = 0;
};
