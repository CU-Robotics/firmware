#pragma once
#include "math.h"
#include <Arduino.h>


/// @brief PIDF filter used in controls. Gains are configurable via K
struct PIDFilter {
    /// @brief proportional gain
    float kp = 0;
    /// @brief integral gain
    float ki = 0;
    /// @brief derivative gain
    float kd = 0;
    /// @brief feedforward gain
    float kf = 0;

    /// @brief integrated error
    float sumError = 0;
    /// @brief previous error
    float prevError = 0;

    /// @brief target
    float setpoint = 0;
    /// @brief estimate
    float measurement = 0;
    
    /// @brief whether to wrap error value
    bool wrap = false;
    /// @brief wrapping min value
    float wrap_min = 0;
    /// @brief wrapping max value
    float wrap_max = 0;

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
        float output = (kp * error) + (kd * ((error - prevError) / dt)) + kf;
        // + (K[1] * sumError)
        // + (K[2] * ((error - prevError) / dt));
        // + (K[3] * feedForward);
        prevError = error;
        if (fabs(output) > 1.0 && bound) output /= fabs(output);
        return output;
    }

    /// @brief set the pidf gains
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    /// @param kf feedforward gain
    void set_gains(float kp, float ki, float kd, float kf) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->kf = kf;
    }
};