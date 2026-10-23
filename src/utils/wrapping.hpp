#ifndef WRAPPING_H
#define WRAPPING_H

namespace Utils {

// Wrap into [min, max), preserving values already in range.
// Requires finite inputs and min < max.
float wrap(float value, float min, float max);

} // namespace Utils

#endif // WRAPPING_H
