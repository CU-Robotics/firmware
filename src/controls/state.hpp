#include "math.h"

#ifndef STATE_H
#define STATE_H

#define STATE_LEN 7

struct State {
    public:
        // {x, y, theta (chassis angle), psi (pitch angle), phi (yaw angle), feed, flywheel}

        // "Desired" state
        float pos[STATE_LEN] = {0.0};
        float vel[STATE_LEN] = {0.0};
        float accel[STATE_LEN] = {0.0};

        // Unfiltered "desired" state
        float unfilt_pos[STATE_LEN] = {0.0};
        float unfilt_vel[STATE_LEN] = {0.0};
        float unfilt_accel[STATE_LEN] = {0.0};

        // Rate limits for "desired" state
        float pos_rate_lim[STATE_LEN] = {0.0};
        float vel_rate_lim[STATE_LEN] = {0.0};
        float accel_rate_lim[STATE_LEN] = {0.0};

        // "Estimated" state
        float est_pos[STATE_LEN] = {0.0};
        float est_vel[STATE_LEN] = {0.0};
        float est_accel[STATE_LEN] = {0.0};

        void setPos(float in_pos[STATE_LEN], float dt) {
            unfilt_pos = in_pos;
            applyRateLimit(unfilt_pos, pos, pos_rate_lim, dt);
        }

        void setVel(float in_vel[STATE_LEN], float dt) {
            unfilt_vel = in_vel;
            applyRateLimit(unfilt_vel, vel, vel_rate_lim, dt);
        }

        void setAccel(float in_accel[STATE_LEN], float dt) {
            unfilt_accel = in_accel;
            applyRateLimit(unfilt_accel, accel, accel_rate_lim, dt);
        }

    private:
        void applyRateLimit(float* unfilt, float* filt, float* rate_lim, float dt) {
            for (int i = 0; i < STATE_LEN; i++) {
                if (rate_lim[i] == 0.0) filt[i] = unfilt[i];
                else {
                    if (fabs(unfilt[i] - filt[i]) > rate_lim[i] * dt) {
                        filt[i] += unfilt[i] - filt[i] > 0 ? rate_lim[i] * dt : -rate_lim[i] * dt;
                    }
                }
            }
        }
};

#endif