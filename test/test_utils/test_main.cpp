#include <Arduino.h>
#include <unity.h>

#include "utils/vector_math.hpp"
#include "utils/wrapping.hpp"

// Wrap tests

void test_wrap_basic() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, Utils::wrap(5.0f, 0.0f, 10.0f));
}

void test_wrap_at_max_clamps_to_min() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, Utils::wrap(10.0f, 0.0f, 10.0f));
}

void test_wrap_above_max() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, Utils::wrap(11.0f, 0.0f, 10.0f));
}

void test_wrap_below_min() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 9.0f, Utils::wrap(-1.0f, 0.0f, 10.0f));
}

void test_wrap_angle_zero() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, Utils::wrap(0.0f, 0.0f, 2.0f * M_PI));
}

void test_wrap_angle_full_rotation() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, Utils::wrap(2.0f * M_PI, 0.0f, 2.0f * M_PI));
}

void test_wrap_angle_negative() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f * M_PI, Utils::wrap(1.5f * M_PI, 0.0f, 2.0f * M_PI));
}

void test_wrap_negative_range() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -M_PI + 0.1f, Utils::wrap(M_PI + 0.1f, -M_PI, M_PI));
}

// Matrix math tests

void test_determinant_3x3() {
    float mat[3][3] = {
        {6, 4, 2},
        {7, 1, 1},
        {9, 2, 0}
    };

    float result = Utils::determinantOfMatrix(mat);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 34.0f, result);
}

void test_rotate_vector_3d() {
    float unitVec[3] = {0.0f, 0.0f, 1.0f};
    float input[3] = {6.0f, 7.0f, 9.0f};
    float output[3];

    float theta = PI / 2.0f;

    Utils::rotateVector3D(unitVec, input, theta, output);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -7.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  6.0f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  9.0f, output[2]);
}

void test_solve_system_all_same() {
    float coeff[3][4] = {
        {67.0f, 67.0f, 67.0f, 67.0f},
        {67.0f, 67.0f, 67.0f, 67.0f},
        {67.0f, 67.0f, 67.0f, 67.0f}
    };
    float output[3];

    Utils::solveSystem(coeff, output);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, output[2]);
}

void test_solve_system_unique_solution() {
    float coeff[3][4] = {
        {6.0f, 7.0f, 9.0f, 1.0f},
        {6.0f, 0.0f, 0.0f, 5.0f},
        {6.0f, 7.0f, 6.0f, 7.0f}
    };
    float output[3];

    Utils::solveSystem(coeff, output);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  5.0f/6.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  2.0f,      output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -2.0f,      output[2]);
}

void setup() {
    delay(2000);

    UNITY_BEGIN();

    // Wrap tests
    RUN_TEST(test_wrap_basic);
    RUN_TEST(test_wrap_at_max_clamps_to_min);
    RUN_TEST(test_wrap_above_max);
    RUN_TEST(test_wrap_below_min);
    RUN_TEST(test_wrap_angle_zero);
    RUN_TEST(test_wrap_angle_full_rotation);
    RUN_TEST(test_wrap_angle_negative);
    RUN_TEST(test_wrap_negative_range);

    // Matrix Math tests
    RUN_TEST(test_determinant_3x3);
    RUN_TEST(test_rotate_vector_3d);
    RUN_TEST(test_solve_system_all_same);
    RUN_TEST(test_solve_system_unique_solution);
    UNITY_END();
}

void loop() {} // Linker Won't like if we don't have this (only for files with Arduino)