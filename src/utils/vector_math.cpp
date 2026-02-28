#include "vector_math.hpp"

void rotate2D(float* v, float* v_tf, float angle) {
	v_tf[0] = (v[0] * cos(angle)) - (v[1] * sin(angle));
	v_tf[1] = (v[0] * sin(angle)) + (v[1] * cos(angle));
}

float vectorProduct(float* a, float* b, int n) {
	int product = 0;
	for (int i = 0; i < n; i++) {
		product += a[i] * b[i];
	}
	return product;
}

float crossProduct2D(float* a, float* b) {
	return (a[0] * b[1]) - (a[1] * b[0]);
}

void weightedVectorAddition(float* a, float* b, float k1, float k2, int n, float* output) {
	for (int i = 0; i < n; i++) {
		output[i] = (k1 * a[i]) + (k2 * b[i]);
	}
}

void nWeightedVectorAddition(float* a, float* b, float* k1, float* k2, int n, float* output) {
	for (int i = 0; i < n; i++) {
		output[i] = (k1[i] * a[i]) + (k2[i] * b[i]);
	}
}

#ifdef UNIT_TEST
#include <doctest/doctest.h>

TEST_CASE("rotate2D rotates a unit x vector by 90 degrees") {
	float input[2] = {1.0f, 0.0f};
	float output[2] = {0.0f, 0.0f};

	rotate2D(input, output, static_cast<float>(M_PI_2));
	CHECK(output[0] == doctest::Approx(0.0f));
	CHECK(output[1] == doctest::Approx(1.0f));
}
#endif
