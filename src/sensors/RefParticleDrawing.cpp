#include "RefParticleDrawing.hpp"

RefParticleDrawing::RefParticleDrawing(RefDrawing &draw) : draw(draw) {}

bool RefParticleDrawing::make_particle(const char *name, uint16_t center_x, uint16_t center_y) {
    return draw.draw_circle(name, center_x, center_y, PARTICLE_RADIUS, PARTICLE_WIDTH);
}
