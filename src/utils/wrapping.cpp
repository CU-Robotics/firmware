#include "wrapping.hpp"

float wrapAngle(float angle) {
	while (angle >= M_PI) angle -= 2 * M_PI;
	while (angle <= -M_PI) angle += 2 * M_PI;
	return angle;
}


#ifdef UNIT_TEST
#include <doctest/doctest.h>

TEST_CASE("wrapAngle wraps representative angles") {
	CHECK(wrapAngle(0.0f) == doctest::Approx(0.0f));
	CHECK(wrapAngle(2.0f * static_cast<float>(M_PI)) == doctest::Approx(0.0f));
	CHECK(wrapAngle(1.5f * static_cast<float>(M_PI)) == doctest::Approx(-0.5f * static_cast<float>(M_PI)));
}

#endif
