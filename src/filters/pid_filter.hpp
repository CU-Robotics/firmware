#include "math.h"

#ifndef PID_FILTER_H
#define PID_FILTER_H

struct PIDFilter {
    float K[4] = {0.0}; // P, I, D, F
    float sumError;
    float prevError;

    float setpoint;
    float measurement;
    float feedForward;

    float filter(float dt) {
        float error = setpoint - measurement;
        sumError += error * dt;
        float output = (K[0] * error) + (K[2] * ((error - prevError) / dt));
            // + (K[1] * sumError)
            // + (K[2] * ((error - prevError) / dt));
            // + (K[3] * feedForward);
        prevError = error;
        if (fabs(output) > 1.0) output /= fabs(output);
        return output;
    }
};

#endif // PID_FILTER_H