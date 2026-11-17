#ifndef MATH_CONSTANTS_H
#define MATH_CONSTANTS_H

// Single-precision versions of the Arduino PI, HALF_PI, TWO_PI and M_PI macros.
// Those macros are double literals, so `float_value * PI` is computed in double,
// which the Teensy 4.1 FPU (single precision) does in software. Use these in
// float math instead.

/// @brief pi as a float
inline constexpr float PI_F = 3.14159265358979323846f;
/// @brief pi / 2 as a float
inline constexpr float HALF_PI_F = 1.57079632679489661923f;
/// @brief 2 * pi as a float
inline constexpr float TWO_PI_F = 6.28318530717958647692f;

#endif // MATH_CONSTANTS_H
