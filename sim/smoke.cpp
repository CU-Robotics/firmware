#include "bridge.h"
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <limits>

static void require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: smoke.cpp:%d: %s\n", line, expression);
        std::exit(EXIT_FAILURE);
    }
}
#define REQUIRE(condition) require(static_cast<bool>(condition), #condition, __LINE__)

// Deliberately simple ABI fixture, not calibrated robot parameters. The Hive
// headless scenario loads repository runtime YAML and exercises the real plant.
static FirmwareSimConfig fixture() {
    FirmwareSimConfig c{};
    for (auto& s : c.states) {
        const float reference[] = {-3.14159265f, 3.14159265f, -10, 10, -20, 20};
        const float physical[] = {-100, 100, -100, 100, -1000, 1000};
        for (int i = 0; i < 6; ++i) { s.reference_limits[i] = reference[i]; s.physical_limits[i] = physical[i]; }
        s.max_controller_error = 100; s.max_error_exceed_time_us = 10000;
    }
    c.states[3].is_wrapping = 1;
    c.yaw.position_gains[0] = 2; c.yaw.velocity_gains[0] = 0.1f;
    c.pitch.position_gains[0] = 2; c.pitch.velocity_gains[0] = 0.1f;
    c.yaw.motor_directions[0] = 1; c.yaw.motor_directions[1] = -1;
    c.pitch.motor_directions[0] = 1; c.pitch.motor_directions[1] = -1;
    c.estimator.yaw_encoder_direction = 1; c.estimator.pitch_encoder_direction = 1;
    c.estimator.pitch_start_angle = 1.5707963f;
    c.estimator.pitch_angle_at_imu_calibration = 1.5707963f;
    c.estimator.yaw_axis_vector[2] = 1; c.estimator.pitch_axis_vector[0] = 1;
    c.estimator.chassis_x_to_motor_rad = 1; c.estimator.chassis_y_to_motor_rad = 1; c.estimator.chassis_rad_to_motor_rad = 1;
    return c;
}
static FirmwareSimOutput step(FirmwareSim* sim, uint64_t time, float gyro = 0) {
    FirmwareSimInput in{}; in.time_us = time; in.armed = 1; in.previous_armed = 1;
    in.target[0] = 0.5f; in.target[3] = 1.5707963f;
    in.sensors[1] = 1.5707963f; in.sensors[4] = gyro;
    FirmwareSimOutput out{}; char err[512];
    const auto status = firmware_sim_step(sim, &in, &out, err, sizeof(err));
    if (status != 0) std::fprintf(stderr, "native smoke fault: %s\n", err);
    REQUIRE(status == 0); return out;
}
int main() {
    char err[512]; auto c = fixture();
    auto* a = firmware_sim_create(&c, err, sizeof(err)); REQUIRE(a);
    auto* b = firmware_sim_create(&c, err, sizeof(err)); REQUIRE(b);
    auto zero = c; zero.yaw.position_gains[0] = 0; zero.yaw.velocity_gains[0] = 0;
    auto* z = firmware_sim_create(&zero, err, sizeof(err)); REQUIRE(z);
    FirmwareSimOutput nominal{}, disabled{};
    for (uint64_t time = 1000; time <= 100000; time += 1000) {
        nominal = step(a, time); const auto repeated = step(b, time);
        disabled = step(z, time);
        for (int i = 0; i < 6; ++i) { REQUIRE(nominal.estimate[i] == repeated.estimate[i]); REQUIRE(nominal.reference[i] == repeated.reference[i]); }
        for (int i = 0; i < 4; ++i) REQUIRE(nominal.motors[i] == repeated.motors[i]);
    }
    REQUIRE(std::abs(nominal.motors[0]) > 0.01f); REQUIRE(disabled.motors[0] == 0);
    const auto biased = step(a, 101000, 0.2f);
    REQUIRE(biased.estimate[0] > nominal.estimate[0]); REQUIRE(biased.estimate[1] > nominal.estimate[1]);
    FirmwareSimInput invalid{}; invalid.time_us = 102000; invalid.armed = 1; invalid.target[0] = std::numeric_limits<float>::quiet_NaN();
    FirmwareSimOutput stopped{};
    REQUIRE(firmware_sim_step(a, &invalid, &stopped, err, sizeof(err)) == 1);
    REQUIRE(stopped.safety_latched == 1); for (float torque : stopped.motors) REQUIRE(torque == 0);
    REQUIRE(firmware_sim_step(a, &invalid, &stopped, err, sizeof(err)) == 1);
    const auto still_running = step(b, 101000); REQUIRE(still_running.safety_latched == 0);
    invalid.time_us = 999; invalid.target[0] = 0;
    REQUIRE(firmware_sim_step(b, &invalid, &stopped, err, sizeof(err)) == -1); REQUIRE(stopped.time_us == 101000);
    c.states[3].is_wrapping = 2; REQUIRE(!firmware_sim_create(&c, err, sizeof(err)));
    std::printf("PASS: actual governor/controller output %.9g; zero-gain output %.9g; gyro estimate %.9g; repeatability/interleaving/fatal latch/time/config validation\n", nominal.motors[0], disabled.motors[0], biased.estimate[0]);
    firmware_sim_destroy(a); firmware_sim_destroy(b); firmware_sim_destroy(z);
}
