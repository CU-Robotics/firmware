#include "controller.hpp"

#include <algorithm>
#include <cmath>

namespace controller {
float compute_power_limit_ratio(float buffer, float limit_thresh, float critical_thresh) {
    // Match the original comparison, including unordered (NaN) inputs.
    if (buffer < limit_thresh) {
        return std::clamp((buffer - critical_thresh) / limit_thresh, 0.0f, 1.0f);
    }
    return 1.0f;
}

MotorVelocities xdrive_mix(float x, float y, float rot, float heading) {
    const float cosine = std::cos(heading);
    const float sine = std::sin(heading);
    return {{
        -x * sine + y * cosine + rot,
        x * cosine + y * sine + rot,
        x * sine - y * cosine + rot,
        -x * cosine - y * sine + rot,
    }};
}

float to_pitch_linkage_angle(float angle) {
    double a = (1087 * std::sin(angle)) / 4;
    double b = (1087 * std::cos(angle)) / 4;
    double c = std::pow(b + 140, 2);
    double d = std::pow(a + 72, 2);
    return std::atan(((75 * d + (a + 147) * (d + c - 150745.0 / 4) + (b + 140) * std::sqrt(162409 * d - std::pow(d + c + 150745.0 / 4, 2) + 162409 * c) + 75 * c + 11305875.0 / 4) / (2 * d + 2 * c) - 75)
        / ((180 * d + (b + 320) * (d + c - 150745.0 / 4) - (a + 72) * std::sqrt(162409 * d - std::pow(d + c + 150745.0 / 4, 2) + 162409 * c) + 180 * c + 6783525) / (2 * d + 2 * c) - 180));
}

float pitch_linkage_gear_ratio(float angle) {
    const float s = std::sin(angle);
    const float c = std::cos(angle);

    const float A = s * 2.7175e2f + 7.2e1f;
    const float B = c * 2.7175e2f + 1.4e2f;
    const float A2 = A * A;
    const float B2 = B * B;
    const float C = A2 + B2;
    const float cA = c * A;
    const float sB = s * B;
    const float D = s * 2.7175e2f + 1.47e2f;
    const float E = cA * 5.435e2f - sB * 5.435e2f;
    const float F = C - 3.768625e4f;
    const float G = A2 * 1.62409e5f;
    const float H = std::sqrt(G - std::pow(C + 3.768625e4f, 2.0f) + B2 * 1.62409e5f);
    const float I = A2 * 2.0f + B2 * 2.0f;
    const float J = cA * 1.087e3f - sB * 1.087e3f;
    const float K = A2 * 7.5e1f + D * F + B * H + B2 * 7.5e1f + 2.82646875e6f;
    const float L = A2 * 1.8e2f + (c * 2.7175e2f + 3.2e2f) * F - A * H + B2 * 1.8e2f + 6.783525e6f;

    return (((E * D + c * F * 2.7175e2f + cA * 4.07625e4f - sB * 4.07625e4f - s * H * 2.7175e2f - (B * (cA * (-8.82692915e7f) + sB * 8.82692915e7f + E * (A2 + B2 + 3.768625e4f) * 2.0f) / H) / 2.0f) / I - 1.0f / (I * I) * J * K)
        / (L / I - 1.8e2f) - ((E * (c * 2.7175e2f + 3.2e2f) - s * (A2 + B2 - 3.768625e4f) * 2.7175e2f - c * H * 2.7175e2f + cA * 9.783e4f - sB * 9.783e4f + (A * (cA * (-8.82692915e7f) + sB * 8.82692915e7f + E * (C + 3.768625e4f) * 2.0f) / H) / 2.0f) / I - 1.0f / (I * I) * J * L)
        / std::pow(L / I - 1.8e2f, 2.0f) * (K / I - 7.5e1f)) / (std::pow(K / I - 7.5e1f, 2.0f) / std::pow(L / I - 1.8e2f, 2.0f) + 1.0f);
}
} // namespace controller

#ifndef UNIT_TEST

// Host tests compile the same calculations without the hardware controllers.
#include "sensors/can/motor.hpp"
#include "sensors/RefSystem.hpp"
#include "utils/system_log.hpp"
#include "utils/safety_state.hpp"

namespace {
/// @brief Unwrap a potentially wrapped error value to maintain continuity across wrap boundaries.
/// @param error The current error value.
/// @param previous_error The previous control loop error value.
/// @param config The state configuration containing wrap range information.
/// @return The unwrapped error maintaining continuity from the previous value.
float unwrap_controller_error(float error, float previous_error, const Cfg::State& config) {
    if (!config.is_wrapping || config.governor_type != Cfg::StateOrder::Position) {
        // Wrapping only applies to position control, and only if the state is configured to wrap
        return error;
    }

    const float wrap_range = config.reference_limits.position.max - config.reference_limits.position.min;
    if (wrap_range <= 0.0f) {
        return error;
    }

    const float delta = error - previous_error;
    const float half_wrap_range = wrap_range * 0.5f;
    if (delta > half_wrap_range) {
        error -= wrap_range;
    } else if (delta < -half_wrap_range) {
        error += wrap_range;
    }

    return error;
}

/// @brief Normalize a wrapped error value into its canonical form within the wrap range.
/// @param error The error value to normalize.
/// @param config The state configuration containing wrap range information.
/// @return The normalized error value constrained to the wrapped position range.
float normalize_wrapped_error(float error, const Cfg::State& config) {
    if (!config.is_wrapping || config.governor_type != Cfg::StateOrder::Position) {
        // Wrapping only applies to position control, and only if the state is configured to wrap
        return error;
    }

    const float wrap_range = config.reference_limits.position.max - config.reference_limits.position.min;
    if (wrap_range <= 0.0f) {
        return error;
    }

    const float half_wrap_range = wrap_range * 0.5f;
    while (error > half_wrap_range) {
        error -= wrap_range;
    }
    while (error < -half_wrap_range) {
        error += wrap_range;
    }

    return error;
}
} // namespace

void Controller::checkControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, ErrorMonitor& monitor) {
    // Skip error checking if in safety mode (robot is not actively controlled)
    if (safety_state.is_safety_mode_active()) {
        // Reset monitor to clean slate when safety mode is active
        monitor.exceeding = false;
        monitor.exceed_start_us = 0;
        monitor.initialized = false;
        monitor.previous_error = 0.0f;
        return;
    }

    const Cfg::State& config = reference_state.config();
    const float reference_value = get_state_error_value(reference_state);
    const float estimate_value = get_state_error_value(estimate_state);

    // Non-finite values indicate broken sensing/state propagation and should fail safe immediately.
    if (!std::isfinite(reference_value) || !std::isfinite(estimate_value)) {
        handleControllerError(controller_name, state_name, reference_state, estimate_state, reference_value - estimate_value);
        return;
    }

    float error = reference_value - estimate_value;
    if (!monitor.initialized) {
        monitor.initialized = true;
        monitor.previous_error = normalize_wrapped_error(error, config);
        error = monitor.previous_error;
    } else {
        error = unwrap_controller_error(error, monitor.previous_error, config);
        monitor.previous_error = error;
    }

    const float abs_error = fabsf(error);
    if (abs_error > config.max_controller_error) {
        if (!monitor.exceeding) {
            monitor.exceeding = true;
            monitor.exceed_start_us = micros();
        }

        // If error exceeds the configured threshold for too long, trigger the error handler
        const uint32_t elapsed_us = static_cast<uint32_t>(micros() - monitor.exceed_start_us);
        if (elapsed_us >= config.max_error_exceed_time_us) {
            handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
        }
    } else {
        monitor.exceeding = false;
        monitor.exceed_start_us = 0;
    }
}

void Controller::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    const Cfg::State& config = reference_state.config();
    safety::safety_procedure(
        "%s: %s controller error exceeded threshold. error=%f threshold=%f",
        controller_name,
        state_name,
        error,
        config.max_controller_error
    );
}

void XDriveController::validate(const RobotStateArray& reference_array, const RobotStateArray& estimate_array) {
    // Our estimates of the chassis states are not accurate enough to run checks against their references.
    // checkControllerError("XDriveController", "Chassis X", reference_array[chassis_x_state], estimate_array[chassis_x_state], chassis_x_error_monitor);
    // checkControllerError("XDriveController", "Chassis Y", reference_array[chassis_y_state], estimate_array[chassis_y_state], chassis_y_error_monitor);
    // checkControllerError("XDriveController", "Chassis Heading", reference_array[chassis_heading_state], estimate_array[chassis_heading_state], chassis_heading_error_monitor);
}

void XDriveController::step(RobotStateArray& reference_array, RobotStateArray& estimate_array, RobotStateArray& target_array) {
    float dt = timer.delta();

    Cfg::StateName drive_states[3]  = { chassis_x_state, chassis_y_state, chassis_heading_state };
    std::shared_ptr<Motor> drive_motors[4] = { chassis_motor_1, chassis_motor_2, chassis_motor_3, chassis_motor_4 };

    if (reference_array[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Position) {

        // High level position controller
        for (int i = 0; i < 2; i++) {
            pidp[i].setpoint = reference_array[drive_states[i]].get_position();
            pidp[i].measurement = estimate_array[drive_states[i]].get_position();

            pidp[i].kp = xy_position_controller.gains.p;
            pidp[i].ki = xy_position_controller.gains.i;
            pidp[i].kd = xy_position_controller.gains.d;
            pidp[i].kf = xy_position_controller.gains.f;

            pidv[i].setpoint = reference_array[drive_states[i]].get_velocity();
            pidv[i].measurement = estimate_array[drive_states[i]].get_velocity();

            pidv[i].kp = xy_velocity_controller.gains.p;
            pidv[i].ki = xy_velocity_controller.gains.i;
            pidv[i].kd = xy_velocity_controller.gains.d;
            pidv[i].kf = reference_array[drive_states[i]].get_velocity();

            outputp[i] = pidp[i].filter(dt, false, false);
            outputv[i] = pidv[i].filter(dt, false, false);
            output[i] = (outputp[i] + outputv[i]) * controller_config.gear_ratios.chassis_x_to_motor_rad;
        }
        pidp[2].setpoint = reference_array[Cfg::StateName::ChassisHeading].get_position();
        pidp[2].measurement = estimate_array[Cfg::StateName::ChassisHeading].get_position();

        pidv[2].setpoint = reference_array[Cfg::StateName::ChassisHeading].get_velocity();
        pidv[2].measurement = estimate_array[Cfg::StateName::ChassisHeading].get_velocity();
        if (reference_array[Cfg::StateName::ChassisHeading].config().governor_type == Cfg::StateOrder::Position) {
            pidp[2].kp = chassis_angle_controller.gains.p;
            pidp[2].ki = chassis_angle_controller.gains.i;
            pidp[2].kd = chassis_angle_controller.gains.d;
            pidv[2].kp = chassis_angular_velocity_controller.gains.p;
            pidv[2].ki = chassis_angular_velocity_controller.gains.i;
            pidv[2].kd = chassis_angular_velocity_controller.gains.d;
            pidv[2].kf = 0;
        } else { // Heading Velocity control
            pidp[2].kp = 0;
            pidp[2].ki = 0;
            pidp[2].kd = 0;
            pidv[2].kp = 0;
            pidv[2].ki = 0;
            pidv[2].kd = 0;
            pidv[2].kf = reference_array[Cfg::StateName::ChassisHeading].get_velocity();
        }
        outputp[2] = pidp[2].filter(dt, false, false);
        outputv[2] = pidv[2].filter(dt, false, false);
        output[2] = (outputp[2] + outputv[2]) * controller_config.gear_ratios.chassis_rad_to_motor_rad;

        float chassis_heading = estimate_array[Cfg::StateName::ChassisHeading].get_position();
        motor_velocity = controller::xdrive_mix(output[0], output[1], output[2], chassis_heading);

        // Power limiting
        float power_limit_ratio = controller::compute_power_limit_ratio(
            ref.ref_data.robot_power_heat.buffer_energy,
            power_buffer_controller.gains.power_buffer_threshold,
            power_buffer_controller.gains.power_buffer_critical_threshold
        );

        float motor_outputs[4];

        // Low level velocity controller
        for (int i = 0; i < 4; i++) {
            PIDFilter pid;
            pid.setpoint = motor_velocity[i];
            pid.measurement = drive_motors[i]->get_state().speed;
            pid.kp = low_level_velocity_controller.gains.p;
            pid.ki = low_level_velocity_controller.gains.i;
            pid.kd = low_level_velocity_controller.gains.d;
            motor_outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
            drive_motors[i]->write_motor_torque(motor_outputs[i]);
        }

    } else if (reference_array[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Velocity) {

        // High level velocity controller
        for (int i = 0; i < 2; i++) {
            pidv[i].kp = xy_velocity_controller.gains.p;
            pidv[i].ki = xy_velocity_controller.gains.i;
            pidv[i].kd = xy_velocity_controller.gains.d;
            pidv[i].kf = reference_array[drive_states[i]].get_velocity();

            pidv[i].setpoint = reference_array[drive_states[i]].get_velocity();
            pidv[i].measurement = estimate_array[drive_states[i]].get_velocity();
            output[i] = pidv[i].filter(dt, false, false) * controller_config.gear_ratios.chassis_x_to_motor_rad;
        }
        pidp[2].setpoint = reference_array[Cfg::StateName::ChassisHeading].get_position();
        pidp[2].measurement = 0;//estimate_array[Cfg::StateName::ChassisHeading].get_position();  put this back for actual chassis heading feedback control
        pidv[2].setpoint = reference_array[Cfg::StateName::ChassisHeading].get_velocity();
        pidv[2].measurement = 0;//estimate_array[Cfg::StateName::ChassisHeading].get_velocity();
        if (reference_array[Cfg::StateName::ChassisHeading].config().governor_type == Cfg::StateOrder::Position) { 
            pidp[2].kp = chassis_angle_controller.gains.p;
            pidp[2].ki = chassis_angle_controller.gains.i;
            pidp[2].kd = chassis_angle_controller.gains.d;
            pidp[2].kf = chassis_angle_controller.gains.f;
            pidv[2].kp = chassis_angular_velocity_controller.gains.p;
            pidv[2].ki = chassis_angular_velocity_controller.gains.i;
            pidv[2].kd = chassis_angular_velocity_controller.gains.d;
            pidv[2].kf = chassis_angular_velocity_controller.gains.f;
        } else { // Heading Velocity control
            pidp[2].kp = 0;
            pidp[2].ki = 0;
            pidp[2].kd = 0;
            pidv[2].kp = 0;
            pidv[2].ki = 0;
            pidv[2].kd = 0;
            pidv[2].kf = reference_array[Cfg::StateName::ChassisHeading].get_velocity();
        }
        outputp[2] = pidp[2].filter(dt, false, false);
        outputv[2] = pidv[2].filter(dt, false, false);
        output[2] = (outputp[2] + outputv[2]) * controller_config.gear_ratios.chassis_rad_to_motor_rad;

        float chassis_heading = estimate_array[Cfg::StateName::ChassisHeading].get_position();

        // Convert to motor velocities
        motor_velocity = controller::xdrive_mix(output[0], output[1], output[2], chassis_heading);
        // Power limiting
        float power_limit_ratio = controller::compute_power_limit_ratio(
            ref.ref_data.robot_power_heat.buffer_energy,
            power_buffer_controller.gains.power_buffer_threshold,
            power_buffer_controller.gains.power_buffer_critical_threshold
        );

        float motor_outputs[4];

        // Low level velocity controller
        for (int i = 0; i < 4; i++) {
            PIDFilter pid;
            pid.setpoint = motor_velocity[i];
            pid.measurement = drive_motors[i]->get_state().speed;
            pid.kp = low_level_velocity_controller.gains.p;
            pid.ki = low_level_velocity_controller.gains.i;
            pid.kd = low_level_velocity_controller.gains.d;
            motor_outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
            drive_motors[i]->write_motor_torque(motor_outputs[i]);
        }
    } else {
        SystemLog.error(Subsystem::Controls,"governor type not used for xdrive controller");
    }
}

void XDriveController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void YawController::validate(const RobotStateArray& reference_array, const RobotStateArray& estimate_array) {
    checkControllerError("YawController", "Gimbal Yaw", reference_array[yaw_angle_state], estimate_array[yaw_angle_state], yaw_error_monitor);
}

void YawController::step(RobotStateArray& reference_array, RobotStateArray& estimate_array, RobotStateArray& target_array) {
    float dt = timer.delta();
    float output = 0.0;

    pidp.kp = full_state_position_controller.gains.p;
    pidp.ki = full_state_position_controller.gains.i;
    pidp.kd = full_state_position_controller.gains.d;
    // pidp.kf = full_state_position_controller.gains.f;
    pidp.kf = full_state_position_controller.gains.f * reference_array[yaw_angle_state].get_acceleration() * controller_config.gear_ratios.accel_to_normalized_torque;

    pidv.kp = full_state_velocity_controller.gains.p;
    pidv.ki = full_state_velocity_controller.gains.i;
    pidv.kd = full_state_velocity_controller.gains.d;
    pidv.kf = full_state_velocity_controller.gains.f;

    pidp.setpoint = reference_array[yaw_angle_state].get_position();
    pidp.measurement = estimate_array[yaw_angle_state].get_position();

    pidv.setpoint = reference_array[yaw_angle_state].get_velocity();
    pidv.measurement = estimate_array[yaw_angle_state].get_velocity();

    output += pidp.filter(dt, true, true);
    output += pidv.filter(dt, true, false);

    output = std::clamp(output, -1.0f, 1.0f);

    float motor_outputs[2];

    motor_outputs[0] = controller_config.gear_ratios.motor1_direction * output;
    motor_outputs[1] = controller_config.gear_ratios.motor2_direction * output;

    yaw_motor_1->write_motor_torque(motor_outputs[0]);
    yaw_motor_2->write_motor_torque(motor_outputs[1]);
}

void YawController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void PitchController::validate(const RobotStateArray& reference_array, const RobotStateArray& estimate_array) {
    checkControllerError("PitchController", "Gimbal Pitch", reference_array[pitch_angle_state], estimate_array[pitch_angle_state], pitch_error_monitor);
}

void PitchController::step(RobotStateArray& reference_array, RobotStateArray& estimate_array, RobotStateArray& target_array) {
    float dt = timer.delta();
    float output = 0.0;

    pidp.kp = full_state_position_controller.gains.p;
    pidp.ki = full_state_position_controller.gains.i;
    pidp.kd = full_state_position_controller.gains.d;
    pidp.kf = full_state_position_controller.gains.f * sin(estimate_array[pitch_angle_state].get_position()); 
    
    pidv.kp = full_state_velocity_controller.gains.p;
    pidv.ki = full_state_velocity_controller.gains.i;
    pidv.kd = full_state_velocity_controller.gains.d;
    pidv.kf = full_state_velocity_controller.gains.f;
    pidp.setpoint = reference_array[pitch_angle_state].get_position();
    pidp.measurement = estimate_array[pitch_angle_state].get_position();

    pidv.setpoint = reference_array[pitch_angle_state].get_velocity();
    pidv.measurement = estimate_array[pitch_angle_state].get_velocity();

    output += pidp.filter(dt, true, false);
    output += pidv.filter(dt, true, false);
    output = std::clamp(output, -1.0f, 1.0f);

    float motor_outputs[2]; 

    motor_outputs[0] = controller_config.gear_ratios.motor1_direction * output;
    motor_outputs[1] = controller_config.gear_ratios.motor2_direction * output;

    pitch_motor_1->write_motor_torque(motor_outputs[0]);
    pitch_motor_2->write_motor_torque(motor_outputs[1]);
}

void PitchController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void FlywheelController::validate(const RobotStateArray& reference_array, const RobotStateArray& estimate_array) {
    // checkControllerError("FlywheelController", "Flywheels", reference_array[flywheel_velocity_state], estimate_array[flywheel_velocity_state], flywheel_error_monitor);
}

void FlywheelController::step(RobotStateArray& reference_array, RobotStateArray& estimate_array, RobotStateArray& target_array) {
    float dt = timer.delta();

    pid_high.kp = high_level_velocity_controller.gains.p;
    pid_high.ki = high_level_velocity_controller.gains.i;
    pid_high.kd = high_level_velocity_controller.gains.d;
    pid_high.kf = reference_array[flywheel_velocity_state].get_velocity();

    pid_high.setpoint = reference_array[flywheel_velocity_state].get_velocity();
    pid_high.measurement = estimate_array[flywheel_velocity_state].get_velocity();

    float target_motor_velocity = pid_high.filter(dt, false, false) * controller_config.gear_ratios.ball_to_flywheel_rad;
    
    std::shared_ptr<Motor> flywheel_motors[3] = { flywheel_motor_1, flywheel_motor_2, flywheel_motor_3 };

    for (int i = 0; i < num_flywheel_motors; i++) {
        pid_low.kp = low_level_velocity_controller.gains.p;
        pid_low.ki = low_level_velocity_controller.gains.i;
        pid_low.kd = low_level_velocity_controller.gains.d;
        pid_low.kf = low_level_velocity_controller.gains.f;

        if (!flywheel_motors[i]) {
            continue; // Skip if the motor is not initialized
        }

        pid_low.setpoint = target_motor_velocity * flywheel_directions[i];
        pid_low.measurement = flywheel_motors[i]->get_state().speed;

        flywheel_motors[i]->write_motor_torque(pid_low.filter(dt, true, false));
    }
}

void FlywheelController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void FeederController::validate(const RobotStateArray& reference_array, const RobotStateArray& estimate_array) {
    // checkControllerError("FeederController", "Feeder", reference_array[feeder_position_state], estimate_array[feeder_position_state], feeder_error_monitor);
}

void FeederController::step(RobotStateArray& reference_array, RobotStateArray& estimate_array, RobotStateArray& target_array) {
    float dt = timer.delta();
    pidp.kp = full_state_position_controller.gains.p;
    pidp.ki = full_state_position_controller.gains.i;
    pidp.kd = full_state_position_controller.gains.d;
    pidp.kf = full_state_position_controller.gains.f;

    pidv.kp = full_state_velocity_controller.gains.p;
    pidv.ki = full_state_velocity_controller.gains.i;
    pidv.kd = full_state_velocity_controller.gains.d;
    pidv.kf = full_state_velocity_controller.gains.f;

    pidp.setpoint = reference_array[feeder_position_state].get_position();
    pidp.measurement = estimate_array[feeder_position_state].get_position();

    float outputp = pidp.filter(dt, true, true);
    float output = outputp * controller_config.gear_ratios.feeder_direction;

    feeder_motor->write_motor_torque(output);
}

void FeederController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void LowerFeederController::validate(const RobotStateArray& reference_array, const RobotStateArray& estimate_array) {
    // checkControllerError("LowerFeederController", "Lower Feeder", reference_array[lower_feeder_position_state], estimate_array[lower_feeder_position_state], lower_feeder_error_monitor);
}

void LowerFeederController::step(RobotStateArray& reference_array, RobotStateArray& estimate_array, RobotStateArray& target_array) {
    float dt = timer.delta();
    upper_pidp.kp = upper_position_controller.gains.p;
    upper_pidp.ki = upper_position_controller.gains.i;
    upper_pidp.kd = upper_position_controller.gains.d;
    upper_pidp.kf = upper_position_controller.gains.f * upper_feeder_reference_state[upper_feeder_position_state].get_acceleration() * controller_config.gear_ratios.upper_accel_to_normalized_torque;

    upper_pidv.kp = upper_velocity_controller.gains.p;
    upper_pidv.ki = upper_velocity_controller.gains.i;
    upper_pidv.kd = upper_velocity_controller.gains.d;
    upper_pidv.kf = upper_velocity_controller.gains.f;

    lower_pidp.kp = lower_position_controller.gains.p;
    lower_pidp.ki = lower_position_controller.gains.i;
    lower_pidp.kd = lower_position_controller.gains.d;
    lower_pidp.kf = lower_position_controller.gains.f * reference_array[upper_feeder_position_state].get_acceleration() * controller_config.gear_ratios.lower_accel_to_normalized_torque;

    lower_pidv.kp = lower_velocity_controller.gains.p;
    lower_pidv.ki = lower_velocity_controller.gains.i;
    lower_pidv.kd = lower_velocity_controller.gains.d;
    lower_pidv.kf = lower_velocity_controller.gains.f;

    float sync_threshold = controller_config.gear_ratios.sync_threshold; // balls

    float upper_pos = estimate_array[upper_feeder_position_state].get_position();
    float lower_pos = estimate_array[lower_feeder_position_state].get_position();

    float upper_target_pos = upper_target[upper_feeder_position_state].get_position();
    float lower_target_pos = target_array[upper_feeder_position_state].get_position();

    // Serial.printf("upper target: %f, lower target: %f, upper pos: %f, lower pos: %f, upper reference: %f, lower reference: %f\n", upper_target_pos, lower_target_pos, upper_pos, lower_pos, upper_feeder_reference_state[upper_feeder_position_state].get_position(), reference_array[lower_feeder_position_state].get_position());

    if (upper_target_pos > lower_target_pos) {
        upper_target_pos--;
        upper_target[upper_feeder_position_state].set_position(upper_target_pos);
    }

    if ((lower_target_pos - upper_target_pos > 0.5) && (lower_pos > upper_pos - sync_threshold)) {
        upper_target_pos++;
        upper_target[upper_feeder_position_state].set_position(upper_target_pos);
        target_increase_time = micros();
        timer_active = true;
    }

    if (upper_pos > upper_target_pos - 0.7 && timer_active) {
        // Serial.printf("upper feeder shot, time: %f \n", (micros() - target_increase_time) / 1000.0);
        timer_active = false;
    }

    upper_feeder_reference_state = upper_feeder_reference_governor.step_reference_array(upper_target);

    upper_pidp.setpoint = upper_feeder_reference_state[upper_feeder_position_state].get_position();
    upper_pidp.measurement = upper_pos;

    upper_pidv.setpoint = upper_feeder_reference_state[upper_feeder_position_state].get_velocity();
    upper_pidv.measurement = estimate_array[upper_feeder_position_state].get_velocity();

    lower_pidp.setpoint = reference_array[upper_feeder_position_state].get_position();
    lower_pidp.measurement = lower_pos;

    lower_pidv.setpoint = reference_array[upper_feeder_position_state].get_velocity();
    lower_pidv.measurement = estimate_array[lower_feeder_position_state].get_velocity();

    float upper_outputp = upper_pidp.filter(dt, true, true);
    float upper_outputv = upper_pidv.filter(dt, true, false);
    float upper_output = (upper_outputp + upper_outputv) * controller_config.gear_ratios.upper_feeder_direction;
    upper_output = std::clamp(upper_output, -1.0f, 1.0f);

    float lower_outputp = lower_pidp.filter(dt, true, true);
    float lower_outputv = lower_pidv.filter(dt, true, false);
    float lower_output = (lower_outputp + lower_outputv) * controller_config.gear_ratios.lower_feeder_direction;
    lower_output = std::clamp(lower_output, -1.0f, 1.0f);

    // Serial.printf("Feeder Velocity Setpoint: %f, Measurement: %f, output: %f\n", lower_pidv.setpoint, lower_pidv.measurement, output);
    // Serial.printf("lower feeder reference position: %f, reference velocity: %f, estimate position: %f, estimate velocity: %f\n",
    //                 reference_array[lower_feeder_position_state].get_position(), reference_array[lower_feeder_position_state].get_velocity(),
    //                 estimate_array[lower_feeder_position_state].get_position(), estimate_array[lower_feeder_position_state].get_velocity());
    upper_feeder_motor->write_motor_torque(upper_output);

    near_feeder_motor->write_motor_torque(lower_output);
    far_feeder_motor->write_motor_torque(-lower_output);
}

void ArmJointController::validate(const RobotStateArray& reference_array, const RobotStateArray& estimate_array) {
    checkControllerError("ArmJointController", "Arm Joint", reference_array[joint_state], estimate_array[joint_state], joint_error_monitor);
}

void ArmJointController::step(RobotStateArray& reference_array, RobotStateArray& estimate_array, RobotStateArray& target_array) {
    float dt = timer.delta();
    float output = 0.0;

    pidp.kp = full_state_position_controller.gains.p;
    pidp.ki = full_state_position_controller.gains.i;
    pidp.kd = full_state_position_controller.gains.d;
    pidp.kf = full_state_position_controller.gains.f;

    pidv.kp = full_state_velocity_controller.gains.p;
    pidv.ki = full_state_velocity_controller.gains.i;
    pidv.kd = full_state_velocity_controller.gains.d;
    pidv.kf = full_state_velocity_controller.gains.f;

    float reference_position = reference_array[joint_state].get_position();
    float reference_velocity = reference_array[joint_state].get_velocity();
    float estimate_position = estimate_array[joint_state].get_position();
    float estimate_velocity = estimate_array[joint_state].get_velocity();

    if (through_pitch_linkage) {
        // run the loops on the motor side of the linkage, where the gear ratio changes with the joint angle
        reference_velocity *= controller::pitch_linkage_gear_ratio(reference_position);
        estimate_velocity *= controller::pitch_linkage_gear_ratio(estimate_position);
        reference_position = controller::to_pitch_linkage_angle(reference_position);
        estimate_position = controller::to_pitch_linkage_angle(estimate_position);
        safety::assert_or_safety_procedure(std::isfinite(reference_position) && std::isfinite(estimate_position) && std::isfinite(reference_velocity) && std::isfinite(estimate_velocity),
            "ArmJointController: pitch joint outside the linkage's range (reference %f, estimate %f)", reference_array[joint_state].get_position(), estimate_array[joint_state].get_position());
    }

    pidp.setpoint = reference_position;
    pidp.measurement = estimate_position;

    pidv.setpoint = reference_velocity;
    pidv.measurement = estimate_velocity;

    output += pidp.filter(dt, true, false);
    output += pidv.filter(dt, true, false);
    output = std::clamp(output, -1.0f, 1.0f);

    joint_motor->write_motor_torque(controller_config.gear_ratios.motor1_direction * output);
}
#endif // UNIT_TEST

/*
MATLAB script that generates to_pitch_linkage_angle and pitch_linkage_gear_ratio
%base joint positions
x4 = 180;
y4 = 75;
x0 = 320;
y0 = 147;
%link lengths
l1 = 271.75;
l2 = 201.5;
l3 = 54;
a1 = l3^2 - l2^2;
C = @(a,b,c,d) (a-b).^2 + (c-d).^2;
m = -1;
syms x
eq = atan( ...
    ( y4 - ( ( (sin(x).*l1 + y0).*(a1+C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) - y4.*(a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) + m.*(x4 - (cos(x).*l1 + x0)).*sqrt(4.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0)).*l2.^2 - (a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))).^2) ) / (2.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) ) ) ...
    ./ ...
    ( x4 - ( ( (cos(x).*l1 + x0).*(a1+C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) - x4.*(a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) + m.*((sin(x).*l1 + y0) - y4).*sqrt(4.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0)).*l2.^2 - (a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))).^2) ) / (2.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) ) ) ...
);
simplify(eq)
diff(eq)
*/
