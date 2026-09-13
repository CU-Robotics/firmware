#include "RefDrawing.hpp"

constexpr uint16_t PARTICLE_LEGNTH = 2;
constexpr uint16_t PARTICLE_WIDTH = 2;

class RefParticleDrawing {
    public:
        explicit RefParticleDrawing(RefDrawing &draw);
        bool make_particle();
        void debug_mode();
    private:
        RefDrawing &draw;
        
};