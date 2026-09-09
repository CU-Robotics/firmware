#pragma once

/// @brief Holds the mixed motor velocity targets for an X-drive chassis.
struct MotorVelocities {
    /// @brief Mixer order: v[0..3] maps to controller motor indices [1, 2, 3, 0].
    /// @details Values retain the input velocity units and are not clamped or normalized.
    float v[4];
};

float compute_power_limit_ratio(float buffer, float limit_thresh, float critical_thresh);
MotorVelocities xdrive_mix(float x, float y, float rot, float heading);
float clamp1(float value);
