#include <unity.h>
#include <cstring>
#include "utils/safety_state.hpp"

void test_individual_inputs() {
    host_fake_micros = 0;
    SafetyState s;

    // let gimbal power settle first
    s.evaluate(false, true, false, true);
    host_fake_micros = GIMBAL_POWER_SETTLE_US;
    s.evaluate(false, true, false, true);

    s.evaluate(true, true, false, true);
    TEST_ASSERT_EQUAL(SafetyState::Reason::TRANSMITTER, s.active_reasons());

    s.evaluate(false, false, false, true);
    TEST_ASSERT_EQUAL(SafetyState::Reason::NOT_CONFIGURED, s.active_reasons());

    s.evaluate(false, true, true, true);
    TEST_ASSERT_EQUAL(SafetyState::Reason::SLOW_LOOP, s.active_reasons());

    s.evaluate(false, true, false, false);
    TEST_ASSERT_EQUAL(SafetyState::Reason::GIMBAL_POWER_OFF, s.active_reasons());
}

void test_all_bad_at_once() {
    host_fake_micros = 0;
    SafetyState s;
    s.evaluate(true, false, true, false);

    uint8_t all = SafetyState::Reason::TRANSMITTER |
                  SafetyState::Reason::NOT_CONFIGURED |
                  SafetyState::Reason::SLOW_LOOP |
                  SafetyState::Reason::GIMBAL_POWER_OFF;

    TEST_ASSERT_EQUAL(all, s.active_reasons());
}

void test_gimbal_power_settling() {
    host_fake_micros = 0;
    SafetyState s;
    s.evaluate(false, true, false, true);
    TEST_ASSERT_TRUE(s.active_reasons() & SafetyState::Reason::GIMBAL_POWER_SETTLING);
}

void test_gimbal_settled_and_armed() {
    host_fake_micros = 0;
    SafetyState s;

    s.evaluate(false, true, false, true);
    host_fake_micros = GIMBAL_POWER_SETTLE_US;
    s.evaluate(false, true, false, true);

    TEST_ASSERT_FALSE(s.active_reasons() & SafetyState::Reason::GIMBAL_POWER_SETTLING);
    TEST_ASSERT_TRUE(s.motors_armed());
}

void test_power_reset_restarts_timer() {
    host_fake_micros = 0;
    SafetyState s;

    // arm first
    s.evaluate(false, true, false, true);
    host_fake_micros = GIMBAL_POWER_SETTLE_US;
    s.evaluate(false, true, false, true);
    TEST_ASSERT_TRUE(s.motors_armed());

    // turn off and back on
    s.evaluate(false, true, false, false);
    host_fake_micros = 4000000;
    s.evaluate(false, true, false, true);

    TEST_ASSERT_TRUE(s.active_reasons() & SafetyState::Reason::GIMBAL_POWER_SETTLING);
    TEST_ASSERT_FALSE(s.motors_armed());
}

void test_reasons_to_string() {
    host_fake_micros = 0;
    char buf[SafetyState::REASON_STR_LEN];

    SafetyState::reasons_to_string(SafetyState::Reason::NONE, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("none", buf);

    uint8_t reasons = SafetyState::Reason::TRANSMITTER | SafetyState::Reason::SLOW_LOOP;
    SafetyState::reasons_to_string(reasons, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "transmitter"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "slow-loop"));

    // check small buffer doesn't overflow
    char small[5];
    memset(small, 'A', sizeof(small));
    SafetyState::reasons_to_string(SafetyState::Reason::TRANSMITTER, small, sizeof(small));
    TEST_ASSERT_EQUAL('\0', small[4]);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_individual_inputs);
    RUN_TEST(test_all_bad_at_once);
    RUN_TEST(test_gimbal_power_settling);
    RUN_TEST(test_gimbal_settled_and_armed);
    RUN_TEST(test_power_reset_restarts_timer);
    RUN_TEST(test_reasons_to_string);
    UNITY_END();
}

void loop() {}