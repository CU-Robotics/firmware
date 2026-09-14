#pragma once

#include "RefDrawing.hpp"

/// @brief Particle circle radius in pixels.
constexpr uint16_t PARTICLE_RADIUS = 2;
/// @brief Particle circle line width in pixels.
constexpr uint16_t PARTICLE_WIDTH = 2;

/// @brief Helper for drawing particles on the Player's Client.
class RefParticleDrawing {
  public:
    /// @brief Construct a particle drawing helper.
    /// @param draw Drawing helper, which must outlive this object.
    explicit RefParticleDrawing(RefDrawing &draw);

    /// @brief Add a particle circle on layer 0 using the team color.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param center_x Particle center x-coordinate in pixels.
    /// @param center_y Particle center y-coordinate in pixels.
    /// @return true when the drawing packet is accepted for transmission.
    bool make_particle(const char *name, uint16_t center_x, uint16_t center_y);

    /// @brief Reserved for particle drawing debugging; not yet implemented.
    void debug_mode();

  private:
    /// @brief Drawing helper used to send particle graphics.
    RefDrawing &draw;
};
