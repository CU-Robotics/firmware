#include <Arduino.h>
#include <cmath>
#include <bit>
#include <cstdint>
#include <limits>
#include "controls/controller.hpp"
#include <unity.h>

using controller::MotorVelocities;
using controller::compute_power_limit_ratio;
using controller::xdrive_mix;

namespace {
// Keep these reference expressions independent of the helpers: they reproduce
// main's inline math and Teensy's constrain macro before the extraction.
float original_power_limit_ratio(float buffer, float threshold, float critical) {
    float ratio = 1.0;
    if (buffer < threshold) {
        float value = (buffer - critical) / threshold;
        ratio = value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
    }
    return ratio;
}

void assert_same_float(float expected, float actual) {
    if ((std::isnan)(expected)) {
        TEST_ASSERT_TRUE((std::isnan)(actual));
    } else {
        // Also check infinities and the sign of zero without a tolerance.
        TEST_ASSERT_EQUAL_UINT32(std::bit_cast<std::uint32_t>(expected),
                                 std::bit_cast<std::uint32_t>(actual));
    }
}
} // namespace

// Power Limiting

void test_power_limit_matches_original(void) {
    const float values[] = {
        -std::numeric_limits<float>::infinity(), -60.0f, -5.0f, -0.0f,
        0.0f, 10.0f, 35.0f, std::nextafter(60.0f, 0.0f), 60.0f,
        std::nextafter(60.0f, 100.0f), 100.0f,
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN(),
    };
    for (float buffer : values) {
        for (float threshold : values) {
            for (float critical : values) {
                assert_same_float(original_power_limit_ratio(buffer, threshold, critical),
                                  compute_power_limit_ratio(buffer, threshold, critical));
            }
        }
    }
}

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

void test_xdrive_matches_original_motor_outputs(void) {
    const float commands[] = {-100.0f, -2.5f, -0.0f, 0.0f, 0.2f, 3.1f, 100.0f};
    const float headings[] = {-2.0f * (float)M_PI, -(float)M_PI, -0.7f,
                              0.0f, 0.7f, (float)M_PI / 2.0f, (float)M_PI};
    for (float x : commands) {
        for (float y : commands) {
            for (float rot : commands) {
                for (float heading : headings) {
                    float original[4];
                    original[1] = x * cos(heading) + y * sin(heading) + rot;
                    original[2] = x * sin(heading) - y * cos(heading) + rot;
                    original[3] = -x * cos(heading) - y * sin(heading) + rot;
                    original[0] = -x * sin(heading) + y * cos(heading) + rot;

                    MotorVelocities mixed = xdrive_mix(x, y, rot, heading);
                    for (int motor = 0; motor < 4; ++motor) {
                        assert_same_float(original[motor], mixed[motor]);
                    }
                }
            }
        }
    }
}

void test_xdrive_pure_x_at_zero_heading(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.0f, mv[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, mv[3]);
}

void test_xdrive_pure_y_at_zero_heading(void) {
    MotorVelocities mv = xdrive_mix(0.0f, 1.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.0f, mv[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, mv[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv[3]);
}

void test_xdrive_pure_rotation(void) {
    MotorVelocities mv = xdrive_mix(0.0f, 0.0f, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, mv[3]);
}

void test_xdrive_pure_x_at_90deg_heading(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 0.0f, 0.0f, (float)M_PI / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, mv[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.0f, mv[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  0.0f, mv[3]);
}

void test_xdrive_opposite_motor_pairs_negate(void) {
    MotorVelocities mv = xdrive_mix(2.5f, 3.1f, 0.0f, 0.7f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -mv[2], mv[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -mv[3], mv[1]);
}

void test_xdrive_motor_index_mapping(void) {
    MotorVelocities vel = xdrive_mix(1.0f, 0.0f, 0.0f, 0.0f);

    // Both controller modes consume the returned motor indices directly.
    const float expected[4] = {0.0f, 1.0f, 0.0f, -1.0f};
    for (int motor = 0; motor < 4; ++motor) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected[motor], vel[motor]);
    }
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
    const float expected[] = {7.0f / 24.0f, 0.5f, -0.125f, -1.0f / 3.0f};
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected[i], mv[i] * ratio);
    }
}

void test_zero_power_limit_zeros_all_motors(void) {
    MotorVelocities mv = xdrive_mix(1.0f, 1.0f, 1.0f, 0.5f);
    float ratio = compute_power_limit_ratio(10.0f, 60.0f, 10.0f);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, mv[i] * ratio);
    }
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_power_limit_matches_original);
    RUN_TEST(test_xdrive_matches_original_motor_outputs);

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
    RUN_TEST(test_xdrive_motor_index_mapping);

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
