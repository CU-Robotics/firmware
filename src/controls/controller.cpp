#include "controller.hpp"
#include "robot_state.hpp"
#include "sensors/RefSystem.hpp"

void XDriveController::step(RobotStateMap reference_map, RobotStateMap estimate_map, float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();

    StateName drive_states[3]  = { StateName::ChassisX, StateName::ChassisY, StateName::ChassisHeading };
    C620& drive_motors[4] = { drive_motor_1, drive_motor_2, drive_motor_3, drive_motor_4 };

    if (reference_map[StateName::ChassisX].config().governor_type == StateOrder::Position) {

        // High level position controller
        for (int i = 0; i < 2; i++) {
            pidp[i].setpoint = reference_map[drive_states[i]].get_position();
            pidp[i].measurement = estimate_map[drive_states[i]].get_position();

            pidp[i].K[0] = xy_position_controller.gains.p;
            pidp[i].K[1] = xy_position_controller.gains.i;
            pidp[i].K[2] = xy_position_controller.gains.d;
            pidp[i].K[3] = xy_position_controller.gains.f;

            pidv[i].setpoint = reference_map[drive_states[i]].get_velocity();
            pidv[i].measurement = estimate_map[drive_states[i]].get_velocity();

            pidv[i].K[0] = xy_velocity_controller.gains.p;
            pidv[i].K[1] = xy_velocity_controller.gains.i;
            pidv[i].K[2] = xy_velocity_controller.gains.d;
            pidv[i].K[3] = reference_map[drive_states[i]].get_velocity();

            outputp[i] = pidp[i].filter(dt, false, false);
            outputv[i] = pidv[i].filter(dt, false, false);
            output[i] = (outputp[i] + outputv[i]) * controller_config.gear_ratios.chassis_x_to_motor_rad;
        }
        pidp[2].setpoint = reference_map[StateName::ChassisHeading].get_position();
        pidp[2].measurement = estimate_map[StateName::ChassisHeading].get_position();

        pidv[2].setpoint = reference_map[StateName::ChassisHeading].get_velocity();
        pidv[2].measurement = estimate_map[StateName::ChassisHeading].get_velocity();
        if (reference_map[StateName::ChassisHeading].config().governor_type == StateOrder::Position) {
            pidp[2].K[0] = chassis_angle_controller.gains.p;
            pidp[2].K[1] = chassis_angle_controller.gains.i;
            pidp[2].K[2] = chassis_angle_controller.gains.d;
            pidv[2].K[0] = chassis_angular_velocity_controller.gains.p;
            pidv[2].K[1] = chassis_angular_velocity_controller.gains.i;
            pidv[2].K[2] = chassis_angular_velocity_controller.gains.d;
            pidv[2].K[3] = 0;
        } else { // Heading Velocity control
            pidp[2].K[0] = 0;
            pidp[2].K[1] = 0;
            pidp[2].K[2] = 0;
            pidv[2].K[0] = 0;
            pidv[2].K[1] = 0;
            pidv[2].K[2] = 0;
            pidv[2].K[3] = reference_map[StateName::ChassisHeading].get_velocity();
        }
        outputp[2] = pidp[2].filter(dt, false, false);
        outputv[2] = pidv[2].filter(dt, false, false);
        output[2] = (outputp[2] + outputv[2]) * controller_config.gear_ratios.chassis_angle_to_motor_rad;
        float chassis_heading = estimate_map[StateName::ChassisHeading].get_position();

        // Convert to motor velocities
        motor_velocity[1] = output[0] * cos(chassis_heading) + output[1] * sin(chassis_heading) + output[2];
        motor_velocity[2] = output[0] * sin(chassis_heading) - output[1] * cos(chassis_heading) + output[2];
        motor_velocity[3] = -output[0] * cos(chassis_heading) - output[1] * sin(chassis_heading) + output[2];
        motor_velocity[0] = -output[0] * sin(chassis_heading) + output[1] * cos(chassis_heading) + output[2];

        // Serial.printf("1: %f, 2: %f, 3: %f, 4: %f\n", motor_velocity[0], motor_velocity[1], motor_velocity[2], motor_velocity[3]);

        // Power limiting
        float power_buffer = ref->ref_data.robot_power_heat.buffer_energy;
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = power_buffer_controller.gains.power_buffer_threshold;
        float power_buffer_critical_thresh = power_buffer_controller.gains.power_buffer_critical_threshold;
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
        }

        // Low level velocity controller
        for (int i = 0; i < 4; i++) {
            PIDFilter pid;
            pid.setpoint = motor_velocity[i];
            pid.measurement = drive_motors[i].get_state().velocity;
            pid.K[0] = low_level_velocity_controller.gains.p;
            pid.K[1] = low_level_velocity_controller.gains.i;
            pid.K[2] = low_level_velocity_controller.gains.d;
            outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
        }

    } else {

        // High level velocity controller
        for (int i = 0; i < 2; i++) {
            pidv[i].K[0] = xy_velocity_controller.gains.p;
            pidv[i].K[1] = xy_velocity_controller.gains.i;
            pidv[i].K[2] = xy_velocity_controller.gains.d;
            pidv[i].K[3] = reference_map[drive_states[i]].get_velocity();

            pidv[i].setpoint = reference_map[drive_states[i]].get_velocity();
            pidv[i].measurement = estimate_map[drive_states[i]].get_velocity();
            output[i] = pidv[i].filter(dt, false, false) * controller_config.gear_ratios.chassis_x_to_motor_rad;
        }
        pidp[2].setpoint = reference_map[StateName::ChassisHeading].get_position();
        pidp[2].measurement = 0;//estimate_map[StateName::ChassisHeading].get_position();  put this back for actual chassis heading feedback control
        pidv[2].setpoint = reference_map[StateName::ChassisHeading].get_velocity();
        pidv[2].measurement = 0;//estimate_map[StateName::ChassisHeading].get_velocity();
        if (reference_map[StateName::ChassisHeading].config().governor_type == StateOrder::Position) { 
            pidp[2].K[0] = chassis_angle_controller.gains.p;
            pidp[2].K[1] = chassis_angle_controller.gains.i;
            pidp[2].K[2] = chassis_angle_controller.gains.d;
            pidp[2].K[3] = chassis_angle_controller.gains.f;
            pidv[2].K[0] = chassis_angular_velocity_controller.gains.p;
            pidv[2].K[1] = chassis_angular_velocity_controller.gains.i;
            pidv[2].K[2] = chassis_angular_velocity_controller.gains.d;
            pidv[2].K[3] = chassis_angular_velocity_controller.gains.f;
        } else { // Heading Velocity control
            pidp[2].K[0] = 0;
            pidp[2].K[1] = 0;
            pidp[2].K[2] = 0;
            pidv[2].K[0] = 0;
            pidv[2].K[1] = 0;
            pidv[2].K[2] = 0;
            pidv[2].K[3] = reference_map[StateName::ChassisHeading].get_velocity();
        }
        outputp[2] = pidp[2].filter(dt, false, false);
        outputv[2] = pidv[2].filter(dt, false, false);
        output[2] = (outputp[2] + outputv[2]) * controller_config.gear_ratios.chassis_rad_to_motor_rad;
        // Serial.printf("chassis heading output: %f, chassis x output: %f, chassis y output: %f chassis angle:%f\n", output[2], output[0], output[1], estimate[2][0]);
        // Adjust for chassis heading so control is field relative
        float chassis_heading = estimate_map[StateName::ChassisHeading].get_position();
        // Convert to motor velocities
        motor_velocity[0] = output[0] * cos(chassis_heading) + output[1] * sin(chassis_heading) + output[2];
        motor_velocity[1] = output[0] * sin(chassis_heading) - output[1] * cos(chassis_heading) + output[2];
        motor_velocity[2] = -output[0] * cos(chassis_heading) - output[1] * sin(chassis_heading) + output[2];
        motor_velocity[3] = -output[0] * sin(chassis_heading) + output[1] * cos(chassis_heading) + output[2];
        // Serial.printf("motor 0: %f, motor 1: %f, motor 2: %f, motor 3: %f\n", motor_velocity[0], motor_velocity[1], motor_velocity[2], motor_velocity[3]);
        // Power limiting
        float power_buffer = ref->ref_data.robot_power_heat.buffer_energy;
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = power_buffer_controller.gains.power_buffer_threshold;
        float power_buffer_critical_thresh = power_buffer_controller.gains.power_buffer_critical_threshold;
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
        }

        // Low level velocity controller
        for (int i = 0; i < 4; i++) {
            PIDFilter pid;
            pid.setpoint = motor_velocity[i];
            pid.measurement = drive_motors[i].get_state().velocity;
            pid.K[0] = low_level_velocity_controller.gains.p;
            pid.K[1] = low_level_velocity_controller.gains.i;
            pid.K[2] = low_level_velocity_controller.gains.d;
            pid.K[3] = low_level_velocity_controller.gains.f;
            outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
            // Serial.printf("motor %d error: %f output: %f\n", i, -micro_estimate[i][1] + motor_velocity[i], outputs[i]);
        }
    }
}

void XDriveVelocityController::step(RobotStateMap reference_map, RobotStateMap estimate_map, float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();

    StateName drive_states[3]  = { StateName::ChassisX, StateName::ChassisY, StateName::ChassisHeading };

    // High level velocity controller
    for (int i = 0; i < 2; i++) {
        pidv[i].K[0] = xy_velocity_controller.gains.p;
        pidv[i].K[1] = xy_velocity_controller.gains.i;
        pidv[i].K[2] = xy_velocity_controller.gains.d;
        pidv[i].K[3] = reference_map[drive_states[i]].get_velocity();

        pidv[i].setpoint = reference_map[drive_states[i]].get_velocity();
        pidv[i].measurement = estimate_map[drive_states[i]].get_velocity();
        output[i] = pidv[i].filter(dt, false, false) * controller_config.gear_ratios.chassis_x_to_motor_rad;
    }
    pidp[2].setpoint = reference[2][0];
    pidp[2].measurement = 0;//estimate[2][0];
    pidv[2].setpoint = reference[2][1];
    pidv[2].measurement = 0;//estimate[2][1];
    if (reference[2][2] == 1) { // if state [2][2] is 1 (We dont use the accel spot for anything) then chassis heading is position controlled 
        pidp[2].K[0] = chassis_angle_controller.gains.p;
        pidp[2].K[1] = chassis_angle_controller.gains.i;
        pidp[2].K[2] = chassis_angle_controller.gains.d;
        pidp[2].K[3] = chassis_angle_controller.gains.f;
        pidv[2].K[0] = chassis_angular_velocity_controller.gains.p;
        pidv[2].K[1] = chassis_angular_velocity_controller.gains.i;
        pidv[2].K[2] = chassis_angular_velocity_controller.gains.d;
        pidv[2].K[3] = chassis_angular_velocity_controller.gains.f;
    } else {
        pidp[2].K[0] = 0;
        pidp[2].K[1] = 0;
        pidp[2].K[2] = 0;
        pidv[2].K[0] = 0;
        pidv[2].K[1] = 0;
        pidv[2].K[2] = 0;
        pidv[2].K[3] = reference[2][1];
    }
    outputp[2] = pidp[2].filter(dt, false, false);
    outputv[2] = pidv[2].filter(dt, false, false);
    output[2] = (outputp[2] + outputv[2]) * controller_config.gear_ratios.chassis_rad_to_motor_rad;
    // Serial.printf("chassis heading output: %f, chassis x output: %f, chassis y output: %f chassis angle:%f\n", output[2], output[0], output[1], estimate[2][0]);
    // Adjust for chassis heading so control is field relative
    float chassis_heading = estimate[2][0];
    // Convert to motor velocities
    motor_velocity[0] = output[0] * cos(chassis_heading) + output[1] * sin(chassis_heading) + output[2];
    motor_velocity[1] = output[0] * sin(chassis_heading) - output[1] * cos(chassis_heading) + output[2];
    motor_velocity[2] = -output[0] * cos(chassis_heading) - output[1] * sin(chassis_heading) + output[2];
    motor_velocity[3] = -output[0] * sin(chassis_heading) + output[1] * cos(chassis_heading) + output[2];
    // Serial.printf("motor 0: %f, motor 1: %f, motor 2: %f, motor 3: %f\n", motor_velocity[0], motor_velocity[1], motor_velocity[2], motor_velocity[3]);
    // Power limiting
    float power_buffer = ref->ref_data.robot_power_heat.buffer_energy;
    float power_limit_ratio = 1.0;
    float power_buffer_limit_thresh = power_buffer_controller.gains.power_buffer_threshold;
    float power_buffer_critical_thresh = power_buffer_controller.gains.power_buffer_critical_threshold;
    if (power_buffer < power_buffer_limit_thresh) {
        power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
    }

    // Low level velocity controller
    for (int i = 0; i < 4; i++) {
        PIDFilter pid;
        pid.setpoint = motor_velocity[i];
        pid.measurement = micro_estimate[i][1];
        pid.K[0] = low_level_velocity_controller.gains.p;
        pid.K[1] = low_level_velocity_controller.gains.i;
        pid.K[2] = low_level_velocity_controller.gains.d;
        pid.K[3] = low_level_velocity_controller.gains.f;
        outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
        // Serial.printf("motor %d error: %f output: %f\n", i, -micro_estimate[i][1] + motor_velocity[i], outputs[i]);
    }
}

void YawController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();
    float output = 0.0;

    pidp.K[0] = full_state_position_controller.gains.p;
    pidp.K[1] = full_state_position_controller.gains.i;
    pidp.K[2] = full_state_position_controller.gains.d;
    pidp.K[3] = full_state_position_controller.gains.f;
    pidv.K[0] = full_state_velocity_controller.gains.p;
    pidv.K[1] = full_state_velocity_controller.gains.i;
    pidv.K[2] = full_state_velocity_controller.gains.d;
    pidv.K[3] = full_state_velocity_controller.gains.f;

    pidp.setpoint = reference[3][0];
    pidp.measurement = estimate[3][0];

    pidv.setpoint = reference[3][1];
    pidv.measurement = estimate[3][1];

    output += pidp.filter(dt, true, true); // position wraps
    output += pidv.filter(dt, true, false); // no wrap for velocity
    output = constrain(output, -1.0, 1.0);
    outputs[0] = controller_config.gear_ratios.motor1_direction * output;
    outputs[1] = controller_config.gear_ratios.motor2_direction * output;

    // Serial.printf("Yaw est: %f, yaw ref: %f, yaw output: %f\n", estimate[3][0], reference[3][0], output);
}

void PitchController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();
    float output = 0.0;

    pidp.K[0] = full_state_position_controller.gains.p;
    pidp.K[1] = full_state_position_controller.gains.i;
    pidp.K[2] = full_state_position_controller.gains.d;
    pidp.K[3] = full_state_position_controller.gains.f * sin(reference[4][0]);
    
    pidv.K[0] = full_state_velocity_controller.gains.p;
    pidv.K[1] = full_state_velocity_controller.gains.i;
    pidv.K[2] = full_state_velocity_controller.gains.d;
    pidv.K[3] = full_state_velocity_controller.gains.f;
    // Serial.printf("pitch angle: %f\n", estimate[4][0]);
    pidp.setpoint = reference[4][0];
    pidp.measurement = estimate[4][0];

    pidv.setpoint = reference[4][1];
    pidv.measurement = estimate[4][1];

    output += pidp.filter(dt, true, true); // position wraps
    output += pidv.filter(dt, true, false); // no wrap for velocity
    output = constrain(output, -1.0, 1.0);

    outputs[0] = controller_config.gear_ratios.motor1_direction * output;
    outputs[1] = controller_config.gear_ratios.motor2_direction * output;

    // Serial.printf("Pitch est: %f, pitch ref: %f, pitch output: %f\n", estimate[4][0], reference[4][0], output);
}

void FlywheelController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();

    pid_high.K[0] = high_level_velocity_controller.gains.p;
    pid_high.K[1] = high_level_velocity_controller.gains.i;
    pid_high.K[2] = high_level_velocity_controller.gains.d;
    pid_high.K[3] = reference[5][1];

    pid_high.setpoint = reference[5][1];
    pid_high.measurement = estimate[5][1];
    // Serial.printf("waggle graph flywheel %f\n", estimate[5][1]);

    float target_motor_velocity = pid_high.filter(dt, false, false) * controller_config.gear_ratios.flywheel_rad_to_motor_rad;
    for (int i = 0; i < 2; i++) {
        pid_low.K[0] = low_level_velocity_controller.gains.p;
        pid_low.K[1] = low_level_velocity_controller.gains.i;
        pid_low.K[2] = low_level_velocity_controller.gains.d;
        pid_low.K[3] = low_level_velocity_controller.gains.f;

        pid_low.setpoint = i == 0 ? target_motor_velocity * controller_config.gear_ratios.motor1_direction : 
                                    target_motor_velocity * controller_config.gear_ratios.motor2_direction;

        pid_low.measurement = micro_estimate[i + 10][1];
        outputs[i] = pid_low.filter(dt, true, false);
    }

    outputs[0] *= controller_config.gear_ratios.motor1_direction;
    outputs[1] *= controller_config.gear_ratios.motor2_direction;
}

// void FeederController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
//     float dt = timer.delta();
    
//     pid_high.K[0] = gains[0];
//     pid_high.K[1] = gains[1];
//     pid_high.K[2] = gains[2];
//     pid_high.K[3] = reference[6][1];

//     pid_high.setpoint = reference[6][1];
//     pid_high.measurement = estimate[6][1];
//     float output = pid_high.filter(dt, false, false)*gear_ratios[0];
//     pid_low.K[0] = gains[3];
//     pid_low.K[1] = gains[4];
//     pid_low.K[2] = gains[5];
//     pid_low.K[3] = output * gear_ratios[1];

//     pid_low.setpoint = output;
//     pid_low.measurement = micro_estimate[12][1];
//     output = pid_low.filter(dt, true, false);
//     // Serial.printf("Feeder output: %f, ref: %f, FF: %f\n", output, pid_low.setpoint-micro_estimate[12][1], pid_low.K[3]);
//     outputs[0] = output;
// }


void FeederController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();

    pidp.K[0] = full_state_position_controller.gains.p;
    pidp.K[1] = full_state_position_controller.gains.i;
    pidp.K[2] = full_state_position_controller.gains.d;
    pidp.K[3] = full_state_position_controller.gains.f;

    pidv.K[0] = full_state_velocity_controller.gains.p;
    pidv.K[1] = full_state_velocity_controller.gains.i;
    pidv.K[2] = full_state_velocity_controller.gains.d;
    pidv.K[3] = full_state_velocity_controller.gains.f;
    

    pidp.setpoint = reference[6][0]; // 1st index = position
    pidp.measurement = estimate[6][0];
    // Serial.printf("reference: %f, estimate: %f\n", reference[6][0], estimate[6][0]);
    // pidv.setpoint = reference[7][1];
    // pidv.measurement = estimate[7][1];

    float outputp = pidp.filter(dt, true, true);
    // Serial.printf("waggle graph outputp: %f\n", outputp);
    // float outputv = pidv.filter(dt, true, false);
    // float output = outputp + outputv;

    outputs[0] = outputp * controller_config.gear_ratios.feeder_direction; // negative because the feeder motor is reversed
}