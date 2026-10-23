#include "wrapping.hpp"
#include <math.h>

namespace Utils {

float wrap(float value, float min, float max) {
	if (value >= min && value < max) return value;

	const double range = static_cast<double>(max) - min;
	double offset = fmod(static_cast<double>(value) - min, range);

	if (offset < 0.0) offset += range;

	const float result = static_cast<float>(static_cast<double>(min) + offset);

	return result >= max ? min : result;
}

} // namespace Utils
