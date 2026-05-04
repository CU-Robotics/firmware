#include "controller.hpp"
#include "motor.hpp"
#include "sensors/RefSystem.hpp"

namespace {
float unwrap_controller_error(float error, float previous_error, const Cfg::State& config) {
    if (!config.is_wrapping || config.governor_type != Cfg::StateOrder::Position) {
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

float normalize_wrapped_error(float error, const Cfg::State& config) {
    if (!config.is_wrapping || config.governor_type != Cfg::StateOrder::Position) {
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
    if (safety::is_safety_mode_active()) {
        return;
    }

    const Cfg::State& config = reference_state.config();
    const float reference_value = get_state_error_value(reference_state);
    const float estimate_value = get_state_error_value(estimate_state);

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

        const uint32_t elapsed_us = static_cast<uint32_t>(micros() - monitor.exceed_start_us);
        if (config.max_error_exceed_time_us == 0 || elapsed_us >= config.max_error_exceed_time_us) {
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

void XDriveController::step(RobotStateMap& reference_map, RobotStateMap& estimate_map) {
    checkControllerError("XDriveController", "Chassis X", reference_map[chassis_x_state], estimate_map[chassis_x_state], chassis_x_error_monitor);
    checkControllerError("XDriveController", "Chassis Y", reference_map[chassis_y_state], estimate_map[chassis_y_state], chassis_y_error_monitor);
    checkControllerError("XDriveController", "Chassis Heading", reference_map[chassis_heading_state], estimate_map[chassis_heading_state], chassis_heading_error_monitor);
    float dt = timer.delta();

    Cfg::StateName drive_states[3]  = { chassis_x_state, chassis_y_state, chassis_heading_state };
    std::shared_ptr<Motor> drive_motors[4] = { chassis_motor_1, chassis_motor_2, chassis_motor_3, chassis_motor_4 };

    if (reference_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Position) {

        // High level position controller
        for (int i = 0; i < 2; i++) {
            pidp[i].setpoint = reference_map[drive_states[i]].get_position();
            pidp[i].measurement = estimate_map[drive_states[i]].get_position();

            pidp[i].kp = xy_position_controller.gains.p;
            pidp[i].ki = xy_position_controller.gains.i;
            pidp[i].kd = xy_position_controller.gains.d;
            pidp[i].kf = xy_position_controller.gains.f;

            pidv[i].setpoint = reference_map[drive_states[i]].get_velocity();
            pidv[i].measurement = estimate_map[drive_states[i]].get_velocity();

            pidv[i].kp = xy_velocity_controller.gains.p;
            pidv[i].ki = xy_velocity_controller.gains.i;
            pidv[i].kd = xy_velocity_controller.gains.d;
            pidv[i].kf = reference_map[drive_states[i]].get_velocity();

            outputp[i] = pidp[i].filter(dt, false, false);
            outputv[i] = pidv[i].filter(dt, false, false);
            output[i] = (outputp[i] + outputv[i]) * controller_config.gear_ratios.chassis_x_to_motor_rad;
        }
        pidp[2].setpoint = reference_map[Cfg::StateName::ChassisHeading].get_position();
        pidp[2].measurement = estimate_map[Cfg::StateName::ChassisHeading].get_position();

        pidv[2].setpoint = reference_map[Cfg::StateName::ChassisHeading].get_velocity();
        pidv[2].measurement = estimate_map[Cfg::StateName::ChassisHeading].get_velocity();
        if (reference_map[Cfg::StateName::ChassisHeading].config().governor_type == Cfg::StateOrder::Position) {
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
            pidv[2].kf = reference_map[Cfg::StateName::ChassisHeading].get_velocity();
        }
        outputp[2] = pidp[2].filter(dt, false, false);
        outputv[2] = pidv[2].filter(dt, false, false);
        output[2] = (outputp[2] + outputv[2]) * controller_config.gear_ratios.chassis_rad_to_motor_rad;
        float chassis_heading = estimate_map[Cfg::StateName::ChassisHeading].get_position();

        // Convert to motor velocities
        motor_velocity[1] = output[0] * cos(chassis_heading) + output[1] * sin(chassis_heading) + output[2];
        motor_velocity[2] = output[0] * sin(chassis_heading) - output[1] * cos(chassis_heading) + output[2];
        motor_velocity[3] = -output[0] * cos(chassis_heading) - output[1] * sin(chassis_heading) + output[2];
        motor_velocity[0] = -output[0] * sin(chassis_heading) + output[1] * cos(chassis_heading) + output[2];

        // Power limiting
        float power_buffer = ref.ref_data.robot_power_heat.buffer_energy;
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = power_buffer_controller.gains.power_buffer_threshold;
        float power_buffer_critical_thresh = power_buffer_controller.gains.power_buffer_critical_threshold;
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
        }

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

    } else if (reference_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Velocity) {

        // High level velocity controller
        for (int i = 0; i < 2; i++) {
            pidv[i].kp = xy_velocity_controller.gains.p;
            pidv[i].ki = xy_velocity_controller.gains.i;
            pidv[i].kd = xy_velocity_controller.gains.d;
            pidv[i].kf = reference_map[drive_states[i]].get_velocity();

            pidv[i].setpoint = reference_map[drive_states[i]].get_velocity();
            pidv[i].measurement = estimate_map[drive_states[i]].get_velocity();
            output[i] = pidv[i].filter(dt, false, false) * controller_config.gear_ratios.chassis_x_to_motor_rad;
        }
        pidp[2].setpoint = reference_map[Cfg::StateName::ChassisHeading].get_position();
        pidp[2].measurement = 0;//estimate_map[Cfg::StateName::ChassisHeading].get_position();  put this back for actual chassis heading feedback control
        pidv[2].setpoint = reference_map[Cfg::StateName::ChassisHeading].get_velocity();
        pidv[2].measurement = 0;//estimate_map[Cfg::StateName::ChassisHeading].get_velocity();
        if (reference_map[Cfg::StateName::ChassisHeading].config().governor_type == Cfg::StateOrder::Position) { 
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
            pidv[2].kf = reference_map[Cfg::StateName::ChassisHeading].get_velocity();
        }
        outputp[2] = pidp[2].filter(dt, false, false);
        outputv[2] = pidv[2].filter(dt, false, false);
        output[2] = (outputp[2] + outputv[2]) * controller_config.gear_ratios.chassis_rad_to_motor_rad;
        // Serial.printf("chassis heading output: %f, chassis x output: %f, chassis y output: %f chassis angle:%f\n", output[2], output[0], output[1], estimate[2][0]);
        // Adjust for chassis heading so control is field relative
        float chassis_heading = estimate_map[Cfg::StateName::ChassisHeading].get_position();

        // Convert to motor velocities
        motor_velocity[1] = output[0] * cos(chassis_heading) + output[1] * sin(chassis_heading) + output[2];
        motor_velocity[2] = output[0] * sin(chassis_heading) - output[1] * cos(chassis_heading) + output[2];
        motor_velocity[3] = -output[0] * cos(chassis_heading) - output[1] * sin(chassis_heading) + output[2];
        motor_velocity[0] = -output[0] * sin(chassis_heading) + output[1] * cos(chassis_heading) + output[2];
        // Power limiting
        float power_buffer = ref.ref_data.robot_power_heat.buffer_energy;
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = power_buffer_controller.gains.power_buffer_threshold;
        float power_buffer_critical_thresh = power_buffer_controller.gains.power_buffer_critical_threshold;
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
        }

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
            // Serial.printf("motor %d error: %f output: %f\n", i, -micro_estimate[i][1] + motor_velocity[i], outputs[i]);
        }
    } else {
        Serial.printf("governor type not used for xdrive controller");
    }
}

void XDriveController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void YawController::step(RobotStateMap& reference_map, RobotStateMap& estimate_map) {
    checkControllerError("YawController", "Gimbal Yaw", reference_map[yaw_angle_state], estimate_map[yaw_angle_state], yaw_error_monitor);
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

    pidp.setpoint = reference_map[yaw_angle_state].get_position();
    pidp.measurement = estimate_map[yaw_angle_state].get_position();

    pidv.setpoint = reference_map[yaw_angle_state].get_velocity();
    pidv.measurement = estimate_map[yaw_angle_state].get_velocity();

    output += pidp.filter(dt, true, true); // position wraps
    output += pidv.filter(dt, true, false); // no wrap for velocity
    
    output = constrain(output, -1.0, 1.0);


    float motor_outputs[2];

    motor_outputs[0] = controller_config.gear_ratios.motor1_direction * output;
    motor_outputs[1] = controller_config.gear_ratios.motor2_direction * output;

    yaw_motor_1->write_motor_torque(motor_outputs[0]);
    yaw_motor_2->write_motor_torque(motor_outputs[1]);
}

void YawController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void PitchController::step(RobotStateMap& reference_map, RobotStateMap& estimate_map) {
    checkControllerError("PitchController", "Gimbal Pitch", reference_map[pitch_angle_state], estimate_map[pitch_angle_state], pitch_error_monitor);
    float dt = timer.delta();
    float output = 0.0;

    pidp.kp = full_state_position_controller.gains.p;
    pidp.ki = full_state_position_controller.gains.i;
    pidp.kd = full_state_position_controller.gains.d;
    pidp.kf = full_state_position_controller.gains.f * sin(estimate_map[pitch_angle_state].get_position()); 
    
    pidv.kp = full_state_velocity_controller.gains.p;
    pidv.ki = full_state_velocity_controller.gains.i;
    pidv.kd = full_state_velocity_controller.gains.d;
    pidv.kf = full_state_velocity_controller.gains.f;
    pidp.setpoint = reference_map[pitch_angle_state].get_position();
    pidp.measurement = estimate_map[pitch_angle_state].get_position();


    pidv.setpoint = reference_map[pitch_angle_state].get_velocity();
    pidv.measurement = estimate_map[pitch_angle_state].get_velocity();

    output += pidp.filter(dt, true, false); // position wraps
    output += pidv.filter(dt, true, false); // no wrap for velocity
    output = constrain(output, -1.0, 1.0);

    float motor_outputs[2]; 

    motor_outputs[0] = controller_config.gear_ratios.motor1_direction * output;
    motor_outputs[1] = controller_config.gear_ratios.motor2_direction * output;

    pitch_motor_1->write_motor_torque(motor_outputs[0]);
    pitch_motor_2->write_motor_torque(motor_outputs[1]);
}

void PitchController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void FlywheelController::step(RobotStateMap& reference_map, RobotStateMap& estimate_map) {
    checkControllerError("FlywheelController", "Flywheels", reference_map[flywheel_velocity_state], estimate_map[flywheel_velocity_state], flywheel_error_monitor);
    float dt = timer.delta();

    pid_high.kp = high_level_velocity_controller.gains.p;
    pid_high.ki = high_level_velocity_controller.gains.i;
    pid_high.kd = high_level_velocity_controller.gains.d;
    pid_high.kf = reference_map[flywheel_velocity_state].get_velocity();

    pid_high.setpoint = reference_map[flywheel_velocity_state].get_velocity();
    pid_high.measurement = estimate_map[flywheel_velocity_state].get_velocity();

    float target_motor_velocity = pid_high.filter(dt, false, false) * controller_config.gear_ratios.ball_to_flywheel_rad;
    
    std::shared_ptr<Motor> flywheel_motors[2] = { flywheel_motor_1, flywheel_motor_2 };
    float motor_outputs[2];

    for (int i = 0; i < 2; i++) {
        pid_low.kp = low_level_velocity_controller.gains.p;
        pid_low.ki = low_level_velocity_controller.gains.i;
        pid_low.kd = low_level_velocity_controller.gains.d;
        pid_low.kf = low_level_velocity_controller.gains.f;

        pid_low.setpoint = i == 0 ? target_motor_velocity * controller_config.gear_ratios.motor1_direction : 
                                    target_motor_velocity * controller_config.gear_ratios.motor2_direction;

        pid_low.measurement = flywheel_motors[i]->get_state().speed;
        motor_outputs[i] = pid_low.filter(dt, true, false);
    }

    flywheel_motor_1->write_motor_torque(motor_outputs[0]);
    flywheel_motor_2->write_motor_torque(motor_outputs[1]);
}

void FlywheelController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void FeederController::step(RobotStateMap& reference_map, RobotStateMap& estimate_map) {
    checkControllerError("FeederController", "Feeder", reference_map[feeder_position_state], estimate_map[feeder_position_state], feeder_error_monitor);
    float dt = timer.delta();
    pidp.kp = full_state_position_controller.gains.p;
    pidp.ki = full_state_position_controller.gains.i;
    pidp.kd = full_state_position_controller.gains.d;
    pidp.kf = full_state_position_controller.gains.f;

    pidv.kp = full_state_velocity_controller.gains.p;
    pidv.ki = full_state_velocity_controller.gains.i;
    pidv.kd = full_state_velocity_controller.gains.d;
    pidv.kf = full_state_velocity_controller.gains.f;
    
    pidp.setpoint = reference_map[feeder_position_state].get_position();
    pidp.measurement = estimate_map[feeder_position_state].get_position();

    float outputp = pidp.filter(dt, true, true);
    float output = outputp * controller_config.gear_ratios.feeder_direction;

   feeder_motor->write_motor_torque(output);
}

void FeederController::handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) {
    Controller::handleControllerError(controller_name, state_name, reference_state, estimate_state, error);
}

void LowerFeederController::step(RobotStateMap& reference_map, RobotStateMap& estimate_map) {
    float dt = timer.delta();
    pidp.kp = full_state_position_controller.gains.p;
    pidp.ki = full_state_position_controller.gains.i;
    pidp.kd = full_state_position_controller.gains.d;
    pidp.kf = full_state_position_controller.gains.f;

    pidv.kp = full_state_velocity_controller.gains.p;
    pidv.ki = full_state_velocity_controller.gains.i;
    pidv.kd = full_state_velocity_controller.gains.d;
    pidv.kf = full_state_velocity_controller.gains.f;
    

    pidp.setpoint = reference_map[lower_feeder_position_state].get_position();
    pidp.measurement = estimate_map[lower_feeder_position_state].get_position();

    pidv.setpoint = reference_map[lower_feeder_position_state].get_velocity();
    pidv.measurement = estimate_map[lower_feeder_position_state].get_velocity();
    
    float outputp = pidp.filter(dt, true, true);
    float outputv = pidv.filter(dt, true, false);
    float output = (outputp + outputv) * controller_config.gear_ratios.feeder_direction;
    
    // Serial.printf("Feeder Velocity Setpoint: %f, Measurement: %f, output: %f\n", pidv.setpoint, pidv.measurement, output);
    // Serial.printf("lower feeder reference position: %f, reference velocity: %f, estimate position: %f, estimate velocity: %f\n", 
    //                 reference_map[lower_feeder_position_state].get_position(), reference_map[lower_feeder_position_state].get_velocity(),
    //                 estimate_map[lower_feeder_position_state].get_position(), estimate_map[lower_feeder_position_state].get_velocity());
        
    near_feeder_motor->write_motor_torque(output);
    far_feeder_motor->write_motor_torque(-output);
}