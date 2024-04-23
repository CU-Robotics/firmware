#include <math.h>

#ifndef WRAPPING_H
#define WRAPPING_H

float wrapAngle(float angle) {
	while (angle >= M_PI) angle -= 2 * M_PI;
	while (angle <= -M_PI) angle += 2 * M_PI;
	return angle;
}

#endif // WRAPPING_H