#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Explicit host ABI: no Cfg inheritance, bitfields, enums, or Rust memory casts.
 * Angles rad, rates rad/s, accelerations rad/s^2, gyro rad/s, time us.
 * State order: chassis x, chassis y, chassis heading, gimbal yaw, gimbal pitch.
 * Gimbal arrays: yaw position/velocity/acceleration, pitch position/velocity/acceleration.
 */
typedef struct {
    float reference_limits[6]; /* position min/max, velocity min/max, acceleration min/max */
    float physical_limits[6];
    uint32_t governor_type; /* production Cfg::StateOrder: position=0, velocity=1, acceleration=2 */
    uint32_t is_wrapping;
    float max_controller_error;
    uint32_t max_error_exceed_time_us;
} FirmwareSimStateConfig;

typedef struct {
    float position_gains[4]; /* production p, i, d, f */
    float velocity_gains[4];
    int32_t motor_directions[2];
    float accel_to_normalized_torque;
} FirmwareSimControllerConfig;

typedef struct {
    float yaw_encoder_offset;
    float pitch_encoder_offset;
    float yaw_encoder_direction;
    float pitch_encoder_direction;
    float yaw_start_angle;
    float pitch_start_angle;
    float roll_start_angle;
    float pitch_angle_at_imu_calibration;
    float yaw_axis_vector[3];
    float pitch_axis_vector[3];
    float chassis_x_to_motor_rad;
    float chassis_y_to_motor_rad;
    float chassis_rad_to_motor_rad;
    uint32_t has_pitch_imu;
} FirmwareSimEstimatorConfig;

typedef struct {
    FirmwareSimStateConfig states[5];
    FirmwareSimControllerConfig yaw;
    FirmwareSimControllerConfig pitch;
    FirmwareSimEstimatorConfig estimator;
} FirmwareSimConfig;

typedef struct {
    uint64_t time_us; /* exactly previous + 1000; first step is 1000 */
    float target[6];
    float sensors[5]; /* encoder yaw, encoder pitch, gyro X/Y/Z; calibrated raw sensor units */
    uint32_t armed; /* 0 zeroes sinks, holds governor at estimate, resets controllers */
} FirmwareSimInput;

typedef struct {
    uint64_t time_us;
    float estimate[6];
    float reference[6];
    float motors[4]; /* normalized torque: yaw motor1/2, pitch motor1/2 */
    uint32_t safety_latched; /* fatal production checks latch until destroy/recreate */
} FirmwareSimOutput;

typedef struct FirmwareSim FirmwareSim;
/* Allocation occurs only here and destroy. err is always NUL terminated if capacity > 0.
 * Initial target/reference/estimate use configured yaw/pitch start angles.
 * Stationary chassis motor feedback is zero rad/s; only yaw/pitch motors are driven.
 * Native safety validators are actual firmware validators; fatal checks stop this instance.
 */
FirmwareSim* firmware_sim_create(const FirmwareSimConfig* config, char* err, uint32_t err_capacity);
void firmware_sim_destroy(FirmwareSim* sim);
/* 0 success; 1 latched safety; -1 invalid input/time. No exceptions cross this ABI. */
int32_t firmware_sim_step(FirmwareSim* sim, const FirmwareSimInput* input, FirmwareSimOutput* output, char* err, uint32_t err_capacity);
/* Verify the C ABI struct sizes against a consumer's repr(C) declarations. */
void firmware_sim_layout(uint32_t sizes[6]); /* state, controller, estimator, config, input, output */
#ifdef __cplusplus
}
#endif
