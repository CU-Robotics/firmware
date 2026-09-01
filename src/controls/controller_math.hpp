#pragma once

/// @brief Holds the normalized velocity outputs for all four motors of an X-drive chassis.
struct MotorVelocities {
    /// @brief Normalized velocity for each motor, indexed 0-3.
    float v[4];
};

float compute_power_limit_ratio(float buffer, float limit_thresh, float critical_thresh);
MotorVelocities xdrive_mix(float x, float y, float rot, float heading);
float clamp1(float value);
