#include "math.h"
#include <Arduino.h>

#ifndef PID_FILTER_H
#define PID_FILTER_H

#define PI 3.1415926535

struct PIDFilter {
    float K[4] = {0.0}; // P, I, D, F
    float sumError;
    float prevError;

    float setpoint;
    float measurement;
    float feedForward;

    float filter(float dt, bool bound, bool wrap) {
        float error = setpoint - measurement;
        if (error > PI && wrap) error -= 2*PI;
        if (error < -PI && wrap) error += 2*PI;
        // if(wrap) Serial.println(error);
        sumError += error * dt;
        float output = (K[0] * error) + (K[2] * ((error - prevError) / dt)) + K[3];
            // + (K[1] * sumError)
            // + (K[2] * ((error - prevError) / dt));
            // + (K[3] * feedForward);
        prevError = error;
        if (fabs(output) > 1.0 && bound) output /= fabs(output);
        return output;
    }

    void set_K(float gains[4]) {
        for(int i = 0;i < 4; i++) {
            K[i] = gains[i];
        }
    }
};

#endif // PID_FILTER_H