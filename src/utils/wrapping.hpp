#pragma once

namespace Utils {

/// @brief Wrap a value into [min, max), preserving values already in range.
/// @param value Value to wrap.
/// @param min Inclusive lower bound of the range.
/// @param max Exclusive upper bound of the range.
/// @return The wrapped value in [min, max). A value equal to max wraps to min.
/// @pre All inputs must be finite, and min must be less than max.
float wrap(float value, float min, float max);

} // namespace Utils
