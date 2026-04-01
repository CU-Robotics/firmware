#include "wrapping.hpp"
#include <math.h>

namespace Utils {

float wrap(float value, float min, float max) {
	float range = max - min;
	value = min + fmod(value - min, range);
	if (value < min) value += range;
	return value;
}

} // namespace Utils