#include "math.h"
#include <Arduino.h>

#ifndef PID_FILTER_H
#define PID_FILTER_H

/// @brief PIDF filter used in controls. Gains are configurable via K
struct PIDFilter {
    /// @brief gains
    float K[4] = { 0.0 }; // P, I, D, F
    /// @brief integrated error
    float sumError = 0;
    /// @brief previous error
    float prevError = 0;

    /// @brief target
    float setpoint = 0;
    /// @brief estimate
    float measurement = 0;
    /// @brief feedforward component
    float feedForward = 0;

    /// @brief calculate pidf output
    /// @param dt delta time
    /// @param bound bound from -1 to 1
    /// @param wrap wrap at 2*pi
    /// @return pidf output
    float filter(float dt, bool bound, bool wrap) {
        float error = setpoint - measurement;
        if (error > PI && wrap) error -= 2 * PI;
        if (error < -PI && wrap) error += 2 * PI;
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
    
    /// @brief set gains
    /// @param gains new gains
    void set_K(float gains[4]) {
        for (int i = 0;i < 4; i++) {
            K[i] = gains[i];
        }
    }
};

#endif // PID_FILTER_H