#ifndef WRAPPING_H
#define WRAPPING_H

float wrapAngle(float angle) {
	while (angle >= PI) angle -= 2*PI;
	while (angle <= -PI) angle += 2*PI;
	return angle;
}

#endif