#include <Arduino.h>
#include <unity.h>
#include <math.h>

#include "filters/pid_filter.hpp"
#include "filters/lowpass_filter.hpp"

// Low Pass tests

void test_lowpass_passthrough() {
    LowpassFilter lpf(0.0f);

    float output = lpf.filter(5.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, output);
}

void test_lowpass_smoothing() {
    LowpassFilter lpf(0.5f);

    lpf.filter(1.0f);
    float output = lpf.filter(1.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.75f, output);
}

void test_lowpass_convergence() {
    LowpassFilter lpf(0.4f);

    float output = 0.0f;
    for (int i = 0; i < 50; i++) {
        output = lpf.filter(10.0f);
    }

    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 10.0f, output);
}

// PID tests

void test_wrap_around() {
    PIDFilter pid;

    pid.set_gains(1.0f, 0.0f, 0.0f, 0.0f);

    pid.setpoint = 0.1f;
    pid.measurement = 2.0f * PI - 0.1f;

    float output = pid.filter(1.0f, false, true);

    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.2f, output);
}

void test_proportional() {
    PIDFilter pid;

    pid.set_gains(2.0f, 0.0f, 0.0f, 0.0f);

    pid.setpoint = 3.0f;
    pid.measurement = 1.0f;

    float output = pid.filter(1.0f, false, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f, output);
}

void test_derivative() {
    PIDFilter pid;

    pid.set_gains(0.0f, 0.0f, 1.0f, 0.0f);

    pid.setpoint = 0.0f;

    pid.measurement = 4.0f;
    pid.filter(1.0f, false, false);

    pid.measurement = 3.0f;
    float output = pid.filter(1.0f, false, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output);
}

void test_output_bound() {
    PIDFilter pid;

    pid.set_gains(10.0f, 0.0f, 0.0f, 0.0f);

    pid.setpoint = 1.0f;
    pid.measurement = 0.0f;

    float output = pid.filter(1.0f, true, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output);
}

void test_feedforward() {
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

    // PIDF tests
    RUN_TEST(test_wrap_around);
    RUN_TEST(test_proportional);
    RUN_TEST(test_derivative);
    RUN_TEST(test_output_bound);
    RUN_TEST(test_feedforward);

    // Low Pass tests
    RUN_TEST(test_lowpass_passthrough);
    RUN_TEST(test_lowpass_smoothing);
    RUN_TEST(test_lowpass_convergence);

    UNITY_END();
}

void loop() {}