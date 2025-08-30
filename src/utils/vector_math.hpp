#include <math.h>

#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

void rotate2D(float* v, float* v_tf, float angle);

float vectorProduct(float* a, float* b, int n);

float crossProduct2D(float* a, float* b);

void weightedVectorAddition(float* a, float* b, float k1, float k2, int n, float* output);

void nWeightedVectorAddition(float* a, float* b, float* k1, float* k2, int n, float* output);

#endif // VECTOR_MATH_H