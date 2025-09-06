#include "wrapping.hpp"

float wrapAngle(float angle) {
	while (angle >= M_PI) angle -= 2 * M_PI;
	while (angle <= -M_PI) angle += 2 * M_PI;
	return angle;
}