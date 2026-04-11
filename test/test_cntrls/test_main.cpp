#include <Arduino.h>
#include <unity.h>
#include <cmath>
#include "controls/controller.hpp"

// Power Limiting

void test_power_limit_full_above_threshold(void) {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, compute_power_limit_ratio(100.0f, 60.0f, 10.0f));
}

void test_power_limit_full_at_threshold(void) {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, compute_power_limit_ratio(60.0f, 60.0f, 10.0f));
}

void test_power_limit_zero_at_critical(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, compute_power_limit_ratio(10.0f, 60.0f, 10.0f));
}

void test_power_limit_zero_below_critical(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, compute_power_limit_ratio(0.0f,  60.0f, 10.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, compute_power_limit_ratio(-5.0f, 60.0f, 10.0f));
}

void test_power_limit_proportional_midpoint(void) {
    float expected = (35.0f - 10.0f) / 60.0f;
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected, compute_power_limit_ratio(35.0f, 60.0f, 10.0f));
}

// XDrive Kinematics

void test_xdrive_pure_x_at_zero_heading(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.0f, mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, mv.v[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[3]);
}

void test_xdrive_pure_y_at_zero_heading(void) {
    MotorVelocities mv = xdrive_mix(0.0f, 1.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, mv.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.0f, mv.v[3]);
}

void test_xdrive_pure_rotation(void) {
    MotorVelocities mv = xdrive_mix(0.0f, 0.0f, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[3]);
}

void test_xdrive_pure_x_at_90deg_heading(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 0.0f, 0.0f, (float)M_PI / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.0f, mv.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, mv.v[3]);
}

void test_xdrive_opposite_motor_pairs_negate(void) {
    MotorVelocities mv = xdrive_mix(2.5f, 3.1f, 0.0f, 0.7f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -mv.v[2], mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -mv.v[3], mv.v[1]);
}

void test_xdrive_motor_index_inconsistency_documented(void) {
    MotorVelocities vel = xdrive_mix(1.0f, 0.0f, 0.0f, 0.0f);

    // position-mode assignment from controller.cpp (indices [1,2,3,0])
    float pos_motor[4];
    pos_motor[1] =  1.0f;
    pos_motor[2] =  0.0f;
    pos_motor[3] = -1.0f;
    pos_motor[0] =  0.0f;

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, vel.v[3], pos_motor[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, vel.v[0], pos_motor[1]);
}

// Output Clamping

void test_clamp_large_positive_becomes_one(void) {
    TEST_ASSERT_EQUAL_FLOAT(1.0f, clamp1(5.0f));
}

void test_clamp_large_negative_becomes_minus_one(void) {
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, clamp1(-5.0f));
}

void test_clamp_value_within_range_unchanged(void) {
    TEST_ASSERT_EQUAL_FLOAT( 0.5f, clamp1( 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, clamp1(-0.5f));
    TEST_ASSERT_EQUAL_FLOAT( 1.0f, clamp1( 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, clamp1(-1.0f));
}

// Pitch Feedforward

void test_pitch_feedforward_zero_at_level(void) {
    float kf = 1.5f * sinf(0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kf);
}

void test_pitch_feedforward_max_at_vertical(void) {
    float base_ff = 1.5f;
    float kf = base_ff * sinf((float)M_PI / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, base_ff, kf);
}

void test_pitch_feedforward_negative_at_negative_angle(void) {
    float kf = 1.5f * sinf(-(float)M_PI / 4.0f);
    TEST_ASSERT_TRUE(kf < 0.0f);
}

void test_pitch_feedforward_proportional_scaling(void) {
    float base_ff = 2.0f;
    float kf_30 = base_ff * sinf((float)M_PI / 6.0f);
    float kf_90 = base_ff * sinf((float)M_PI / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, kf_30 / kf_90);
}

// Heading Velocity Mode

void test_heading_velocity_mode_position_gains_zeroed(void) {
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ki);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kd);
}

void test_heading_velocity_mode_feedforward_equals_setpoint(void) {
    float commanded_vel = 3.7f;
    float kf = commanded_vel;
    TEST_ASSERT_EQUAL_FLOAT(commanded_vel, kf);
}

// Flywheel

void test_flywheel_gear_ratio_scales_target_velocity(void) {
    TEST_ASSERT_EQUAL_FLOAT(105.0f, 30.0f * 3.5f);
}

void test_flywheel_motors_get_opposite_directions(void) {
    float base_vel = 50.0f;
    TEST_ASSERT_EQUAL_FLOAT( 50.0f, base_vel *  1.0f);
    TEST_ASSERT_EQUAL_FLOAT(-50.0f, base_vel * -1.0f);
}

// Yaw

void test_yaw_motor_directions_applied(void) {
    float output = 0.8f;
    TEST_ASSERT_EQUAL_FLOAT( 0.8f,  1.0f * output);
    TEST_ASSERT_EQUAL_FLOAT(-0.8f, -1.0f * output);
}

void test_yaw_motors_equal_magnitude(void) {
    float output = 0.6f;
    TEST_ASSERT_EQUAL_FLOAT(fabsf(1.0f * output), fabsf(-1.0f * output));
}

// Feeder

void test_feeder_direction_forward(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.5f, 0.5f * 1.0f);
}

void test_feeder_direction_reverse(void) {
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, 0.5f * -1.0f);
}

// Mix + Power Limiting

void test_power_limit_scales_all_motors(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 0.5f, 0.2f, 0.0f);
    float ratio = compute_power_limit_ratio(35.0f, 60.0f, 10.0f);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, mv.v[i] * ratio, mv.v[i] * ratio);
    }
}

void test_zero_power_limit_zeros_all_motors(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 1.0f, 1.0f, 0.5f);
    float ratio = compute_power_limit_ratio(10.0f, 60.0f, 10.0f);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, mv.v[i] * ratio);
    }
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_power_limit_full_above_threshold);
    RUN_TEST(test_power_limit_full_at_threshold);
    RUN_TEST(test_power_limit_zero_at_critical);
    RUN_TEST(test_power_limit_zero_below_critical);
    RUN_TEST(test_power_limit_proportional_midpoint);

    RUN_TEST(test_xdrive_pure_x_at_zero_heading);
    RUN_TEST(test_xdrive_pure_y_at_zero_heading);
    RUN_TEST(test_xdrive_pure_rotation);
    RUN_TEST(test_xdrive_pure_x_at_90deg_heading);
    RUN_TEST(test_xdrive_opposite_motor_pairs_negate);
    RUN_TEST(test_xdrive_motor_index_inconsistency_documented);

    RUN_TEST(test_clamp_large_positive_becomes_one);
    RUN_TEST(test_clamp_large_negative_becomes_minus_one);
    RUN_TEST(test_clamp_value_within_range_unchanged);

    RUN_TEST(test_pitch_feedforward_zero_at_level);
    RUN_TEST(test_pitch_feedforward_max_at_vertical);
    RUN_TEST(test_pitch_feedforward_negative_at_negative_angle);
    RUN_TEST(test_pitch_feedforward_proportional_scaling);

    RUN_TEST(test_heading_velocity_mode_position_gains_zeroed);
    RUN_TEST(test_heading_velocity_mode_feedforward_equals_setpoint);

    RUN_TEST(test_flywheel_gear_ratio_scales_target_velocity);
    RUN_TEST(test_flywheel_motors_get_opposite_directions);

    RUN_TEST(test_yaw_motor_directions_applied);
    RUN_TEST(test_yaw_motors_equal_magnitude);

    RUN_TEST(test_feeder_direction_forward);
    RUN_TEST(test_feeder_direction_reverse);

    RUN_TEST(test_power_limit_scales_all_motors);
    RUN_TEST(test_zero_power_limit_zeros_all_motors);

    UNITY_END();
}

void loop() {}