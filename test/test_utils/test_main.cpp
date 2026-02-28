#include <Arduino.h>
#include <unity.h>

#include "utils/vector_math.hpp"
#include "utils/wrapping.hpp"

void test_rotate2d() {
    float input[2] = {1.0f, 0.0f};
    float output[2] = {0.0f, 0.0f};

    rotate2D(input, output, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, output[1]);
}

void test_wrap_angle() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, wrapAngle(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, wrapAngle(2.0f * static_cast<float>(M_PI)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f * static_cast<float>(M_PI), wrapAngle(1.5f * static_cast<float>(M_PI)));
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_rotate2d);
    RUN_TEST(test_wrap_angle);
    UNITY_END();
}

void loop() {}
