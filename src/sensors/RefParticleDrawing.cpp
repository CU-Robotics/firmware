#include "RefParticleDrawing.hpp"



bool RefParticleDrawing::make_particle(const char *name, u_int16_t center_x, u_int16_t center_y) {
    return draw.draw_circle(name, center_x, center_y, 2, 2, 0,)
}