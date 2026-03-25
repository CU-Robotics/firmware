#include <Arduino.h>
#include <unity.h>
#include <math.h>

#include "filters/pid_filter.hpp"

void test_wrap_around() {
    PIDFilter pid;

    float gains[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    pid.set_K(gains);

    pid.setpoint = 0.1f;
    pid.measurement = 2.0f * PI - 0.1f;

    float output = pid.filter(1.0f, false, true);

    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.2f, output);
}

// Integral test if we ever uncomment it out
/*void test_integral() {
    PIDFilter pid;

    float gains[4] = {0.0f, 1.0f, 0.0f, 0.0f}; // Ki = 1
    pid.set_K(gains);

    pid.setpoint = 1.0f;
    pid.measurement = 0.0f; // error = 1

    float out1 = pid.filter(1.0f, false, false);
    float out2 = pid.filter(1.0f, false, false);
    float out3 = pid.filter(1.0f, false, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, out1);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, out2);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 3.0f, out3);
}*/

void test_derivative() {
    PIDFilter pid;

    float gains[4] = {0.0f, 0.0f, 1.0f, 0.0f};
    pid.set_K(gains);

    pid.setpoint = 0.5f;
    pid.measurement = 4;
    pid.filter(1.0f, false, false);

    pid.measurement = 3.0f;
    float output = pid.filter(1.0f, false, false);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output);
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_wrap_around);
    //RUN_TEST(test_integral);
    RUN_TEST(test_derivative);
    UNITY_END();
}



void loop() {}