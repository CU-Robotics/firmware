#include <Arduino.h>
#include <unity.h>
#include <math.h>

#include "filters/pid_filter.hpp"

void testWrapAround() {
    PIDFilter pid;

    pid.set_gains(1.0f, 0.0f, 0.0f, 0.0f);

    pid.setpoint = 0.1f;
    pid.measurement = 2.0f * PI - 0.1f;

    float output = pid.filter(1.0f, false, true);

    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.2f, output);
}

void testProportional() {
    PIDFilter pid;

    pid.set_gains(2.0f, 0.0f, 0.0f, 0.0f);

    pid.setpoint = 3.0f;
    pid.measurement = 1.0f;

    float output = pid.filter(1.0f, false, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f, output);
}

void testDerivative() {
    PIDFilter pid;

    pid.set_gains(0.0f, 0.0f, 1.0f, 0.0f);

    pid.setpoint = 0.0f;

    pid.measurement = 4.0f;
    pid.filter(1.0f, false, false);

    pid.measurement = 3.0f;
    float output = pid.filter(1.0f, false, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output);
}

void testOutputBound() {
    PIDFilter pid;

    pid.set_gains(10.0f, 0.0f, 0.0f, 0.0f);

    pid.setpoint = 1.0f;
    pid.measurement = 0.0f;

    float output = pid.filter(1.0f, true, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output);
}

void testFeedforward() {
    PIDFilter pid;

    pid.set_gains(0.0f, 0.0f, 0.0f, 0.5f);

    pid.setpoint = 0.0f;
    pid.measurement = 0.0f;

    float output = pid.filter(1.0f, false, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, output);
}

void setup() {
    delay(2000);

    UNITY_BEGIN();

    RUN_TEST(testWrapAround);
    RUN_TEST(testProportional);
    RUN_TEST(testDerivative);
    RUN_TEST(testOutputBound);
    RUN_TEST(testFeedforward);

    UNITY_END();
}

void loop() {}