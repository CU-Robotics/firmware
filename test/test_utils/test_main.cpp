#include <Arduino.h>
#include <unity.h>

#include "utils/vector_math.hpp"
#include "utils/wrapping.hpp"


void test_determinant_3x3() {
    float mat[3][3] = {
        {6, 4, 2},
        {7, 1, 1},
        {9, 2, 0}
    };

    float result = determinantOfMatrix(mat);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 34.0f, result);
}

void test_rotate_vector_3d() {
    float unitVec[3] = {0.0f, 0.0f, 1.0f};
    float input[3] = {6.0f, 7.0f, 9.0f};
    float output[3];

    // 90 degs
    float theta = PI / 2.0f;

    rotateVector3D(unitVec, input, theta, output);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -7.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  6.0f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  9.0f, output[2]);
}


void test_wrap_angle() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, wrapAngle(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, wrapAngle(2.0f * static_cast<float>(M_PI)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f * static_cast<float>(M_PI), wrapAngle(1.5f * static_cast<float>(M_PI)));
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_wrap_angle);
    RUN_TEST(test_determinant_3x3)
    RUN_TEST(test_rotate_vector_3d)
    UNITY_END();
}
