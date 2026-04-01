#include <Arduino.h>
#include <unity.h>

#include "utils/vector_math.hpp"
#include "utils/wrapping.hpp"


void testWrapBasic() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, Utils::wrap(5.0f, 0.0f, 10.0f));
}

void testWrapAtMaxClampsToMin() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, Utils::wrap(10.0f, 0.0f, 10.0f));
}

void testWrapAboveMax() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, Utils::wrap(11.0f, 0.0f, 10.0f));
}

void testWrapBelowMin() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 9.0f, Utils::wrap(-1.0f, 0.0f, 10.0f));
}

void testWrapAngleZero() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, Utils::wrap(0.0f, 0.0f, 2.0f * M_PI));
}

void testWrapAngleFullRotation() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, Utils::wrap(2.0f * M_PI, 0.0f, 2.0f * M_PI));
}

void testWrapAngleNegative() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f * M_PI, Utils::wrap(1.5f * M_PI, 0.0f, 2.0f * M_PI));
}

void testWrapNegativeRange() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -M_PI + 0.1f, Utils::wrap(M_PI + 0.1f, -M_PI, M_PI));
}

void testDeterminant3x3() {
    float mat[3][3] = {
        {6, 4, 2},
        {7, 1, 1},
        {9, 2, 0}
    };

    float result = determinantOfMatrix(mat);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 34.0f, result);
}

void testRotateVector3d() {
    float unitVec[3] = {0.0f, 0.0f, 1.0f};
    float input[3] = {6.0f, 7.0f, 9.0f};
    float output[3];

    float theta = PI / 2.0f;

    rotateVector3D(unitVec, input, theta, output);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -7.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  6.0f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  9.0f, output[2]);
}

void testSolveSystemAllSame() {
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

void testSolveSystemUniqueSolution() {
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
    RUN_TEST(testWrapBasic);
    RUN_TEST(testWrapAtMaxClampsToMin);
    RUN_TEST(testWrapAboveMax);
    RUN_TEST(testWrapBelowMin);
    RUN_TEST(testWrapAngleZero);
    RUN_TEST(testWrapAngleFullRotation);
    RUN_TEST(testWrapAngleNegative);
    RUN_TEST(testWrapNegativeRange);
    RUN_TEST(testDeterminant3x3);
    RUN_TEST(testRotateVector3d);
    RUN_TEST(testSolveSystemAllSame);
    RUN_TEST(testSolveSystemUniqueSolution);
    UNITY_END();
}

void loop() {} // Linker Won't like if we don't have this (only for files with Arduino)
