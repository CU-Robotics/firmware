#pragma once

/// @file controller_math.hpp
/// @brief Math helpers for controller power limiting and motor velocity mixing.

/// @brief Holds the mixed motor velocity targets for an X-drive chassis.
struct MotorVelocities {
    /// @brief Mixer order: v[0..3] maps to controller motor indices [1, 2, 3, 0].
    /// @details Values retain the input velocity units and are not clamped or normalized.
    float v[4];
};

/// @brief Compute the motor output scale from the available power buffer.
/// @param buffer Current power buffer level.
/// @param limit_thresh Buffer level below which limiting starts, in the same units as buffer.
/// @param critical_thresh Buffer offset used in the limiting calculation, in the same units as buffer.
/// @return If buffer is below limit_thresh, (buffer - critical_thresh) / limit_thresh
/// clamped to [0, 1]; otherwise, 1.
float compute_power_limit_ratio(float buffer, float limit_thresh, float critical_thresh);

/// @brief Mix translation and rotation into four X-drive motor velocity targets.
/// @param x Translational velocity input along the reference frame's x axis.
/// @param y Translational velocity input along the reference frame's y axis.
/// @param rot Rotational contribution added to each motor target, in the same units as x and y.
/// @param heading Chassis heading relative to the translation reference frame, in radians.
/// @return Unclamped, unnormalized motor velocity targets in the order documented by MotorVelocities::v.
MotorVelocities xdrive_mix(float x, float y, float rot, float heading);

/// @brief Clamp a controller output to the inclusive range [-1, 1]. Basically uselss just here so I can use this in the testing file instead of the real controller file.
/// @param value Controller output to clamp.
/// @return The input value constrained to [-1, 1]; NaN inputs remain NaN.
float clamp1(float value);
