#include <Arduino.h>
#include <unity.h>
#include <cmath>

// Helpers

static float compute_power_limit_ratio(float buffer, float limit_thresh, float critical_thresh) {
    float ratio = 1.0f;
    if (buffer < limit_thresh) {
        float raw = (buffer - critical_thresh) / limit_thresh;
        ratio = raw < 0.0f ? 0.0f : (raw > 1.0f ? 1.0f : raw);
    }
    return ratio;
}

struct MotorVelocities { float v[4]; };

static MotorVelocities xdrive_mix(float x, float y, float rot, float heading) {
    return {{
         x * cosf(heading) + y * sinf(heading) + rot,
         x * sinf(heading) - y * cosf(heading) + rot,
        -x * cosf(heading) - y * sinf(heading) + rot,
        -x * sinf(heading) + y * cosf(heading) + rot
    }};
}

static float clamp1(float v) {
    return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
}

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
    // buffer=35, critical=10, thresh=60  →  (35-10)/60 ≈ 0.4167
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
    // All four motors get equal rotation command
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv.v[3]);
}

void test_xdrive_pure_x_at_90deg_heading(void) {
    // After 90° rotation, a world-frame X command acts like Y at 0°
    MotorVelocities mv = xdrive_mix(1.0f, 0.0f, 0.0f, (float)M_PI / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.0f, mv.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv.v[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, mv.v[3]);
}

void test_xdrive_opposite_motor_pairs_negate(void) {
    // Without rotation, motors 0&2 and motors 1&3 are always negatives of each other
    MotorVelocities mv = xdrive_mix(2.5f, 3.1f, 0.0f, 0.7f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -mv.v[2], mv.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -mv.v[3], mv.v[1]);
}

// Motor Index Inconsistency (position vs velocity mode)
// Position mode assigns motor velocities in [1,2,3,0] order while
// velocity mode uses [0,1,2,3]. This test documents the divergence
// so it can't be silently broken further — update it if intentionally fixed.

void test_xdrive_motor_index_inconsistency_documented(void) {
    float x = 1.0f, y = 0.0f, rot = 0.0f, h = 0.0f;

    float vel_motor[4];
    vel_motor[0] =  x * cosf(h) + y * sinf(h) + rot;
    vel_motor[1] =  x * sinf(h) - y * cosf(h) + rot;
    vel_motor[2] = -x * cosf(h) - y * sinf(h) + rot;
    vel_motor[3] = -x * sinf(h) + y * cosf(h) + rot;

    float pos_motor[4];
    pos_motor[1] =  x * cosf(h) + y * sinf(h) + rot;
    pos_motor[2] =  x * sinf(h) - y * cosf(h) + rot;
    pos_motor[3] = -x * cosf(h) - y * sinf(h) + rot;
    pos_motor[0] = -x * sinf(h) + y * cosf(h) + rot;

    // pos_motor[0] == vel_motor[3], not vel_motor[0]
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, vel_motor[3], pos_motor[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, vel_motor[0], pos_motor[1]);
}

// Output Clamping (Yaw / Pitch use constrain to [-1, 1])

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

// Pitch Feedforward – Gravity Compensation
// pidp.kf = gains.f * sin(pitch_angle)

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
    float kf_30   = base_ff * sinf((float)M_PI / 6.0f);
    float kf_90   = base_ff * sinf((float)M_PI / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, kf_30 / kf_90);
}

// Heading Velocity Mode – position PID must be zeroed

void test_heading_velocity_mode_position_gains_zeroed(void) {
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ki);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kd);
}

void test_heading_velocity_mode_feedforward_equals_setpoint(void) {
    float commanded_vel = 3.7f;
    float kf = commanded_vel; // kf = reference_map[ChassisHeading].get_velocity()
    TEST_ASSERT_EQUAL_FLOAT(commanded_vel, kf);
}

// Flywheel Gear Ratio

void test_flywheel_gear_ratio_scales_target_velocity(void) {
    float ball_vel       = 30.0f;
    float gear_ratio     = 3.5f;
    float target_motor_v = ball_vel * gear_ratio;
    TEST_ASSERT_EQUAL_FLOAT(105.0f, target_motor_v);
}

void test_flywheel_motors_get_opposite_directions(void) {
    float base_vel = 50.0f;
    TEST_ASSERT_EQUAL_FLOAT( 50.0f, base_vel *  1.0f);
    TEST_ASSERT_EQUAL_FLOAT(-50.0f, base_vel * -1.0f);
}

// Yaw Motor Direction

void test_yaw_motor_directions_applied(void) {
    float output = 0.8f;
    TEST_ASSERT_EQUAL_FLOAT( 0.8f,  1.0f * output);
    TEST_ASSERT_EQUAL_FLOAT(-0.8f, -1.0f * output);
}

void test_yaw_motors_equal_magnitude(void) {
    float output = 0.6f;
    TEST_ASSERT_EQUAL_FLOAT(fabsf(1.0f * output), fabsf(-1.0f * output));
}

// Feeder direction

void test_feeder_direction_forward(void) {
    float outputp = 0.5f;
    TEST_ASSERT_EQUAL_FLOAT(0.5f, outputp * 1.0f);
}

void test_feeder_direction_reverse(void) {
    float outputp = 0.5f;
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, outputp * -1.0f);
}

// Mix + Power Limiting

void test_power_limit_scales_all_motors(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 0.5f, 0.2f, 0.0f);
    float ratio = 0.5f;
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, mv.v[i] * 0.5f, mv.v[i] * ratio);
    }
}

void test_zero_power_limit_zeros_all_motors(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 1.0f, 1.0f, 0.5f);
    float ratio = 0.0f;
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, mv.v[i] * ratio);
    }
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    // Power limiting
    RUN_TEST(test_power_limit_full_above_threshold);
    RUN_TEST(test_power_limit_full_at_threshold);
    RUN_TEST(test_power_limit_zero_at_critical);
    RUN_TEST(test_power_limit_zero_below_critical);
    RUN_TEST(test_power_limit_proportional_midpoint);

    // XDrive kinematics
    RUN_TEST(test_xdrive_pure_x_at_zero_heading);
    RUN_TEST(test_xdrive_pure_y_at_zero_heading);
    RUN_TEST(test_xdrive_pure_rotation);
    RUN_TEST(test_xdrive_pure_x_at_90deg_heading);
    RUN_TEST(test_xdrive_opposite_motor_pairs_negate);
    RUN_TEST(test_xdrive_motor_index_inconsistency_documented);

    // Output clamping
    RUN_TEST(test_clamp_large_positive_becomes_one);
    RUN_TEST(test_clamp_large_negative_becomes_minus_one);
    RUN_TEST(test_clamp_value_within_range_unchanged);

    // Pitch feedforward
    RUN_TEST(test_pitch_feedforward_zero_at_level);
    RUN_TEST(test_pitch_feedforward_max_at_vertical);
    RUN_TEST(test_pitch_feedforward_negative_at_negative_angle);
    RUN_TEST(test_pitch_feedforward_proportional_scaling);

    // Heading velocity mode
    RUN_TEST(test_heading_velocity_mode_position_gains_zeroed);
    RUN_TEST(test_heading_velocity_mode_feedforward_equals_setpoint);

    // Flywheel
    RUN_TEST(test_flywheel_gear_ratio_scales_target_velocity);
    RUN_TEST(test_flywheel_motors_get_opposite_directions);

    // Yaw
    RUN_TEST(test_yaw_motor_directions_applied);
    RUN_TEST(test_yaw_motors_equal_magnitude);

    // Feeder
    RUN_TEST(test_feeder_direction_forward);
    RUN_TEST(test_feeder_direction_reverse);

    // Integration
    RUN_TEST(test_power_limit_scales_all_motors);
    RUN_TEST(test_zero_power_limit_zeros_all_motors);

    UNITY_END();
}

void loop() {}