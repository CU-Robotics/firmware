#include "controller.hpp"

void XDrivePositionController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();
    // High level position controller
    for (int i = 0; i < 2; i++) {
        pidp[i].setpoint = reference[i][0];
        pidp[i].measurement = estimate[i][0];

        pidp[i].K[0] = gains[0];
        pidp[i].K[1] = gains[1];
        pidp[i].K[2] = gains[2];

        pidv[i].setpoint = reference[i][1];
        pidv[i].measurement = estimate[i][1];

        pidv[i].K[0] = gains[3];
        pidv[i].K[1] = gains[4];
        pidv[i].K[2] = gains[5];

        outputp[i] = pidp[i].filter(dt, false, false);
        outputv[i] = pidv[i].filter(dt, false, false);
        output[i] = (outputp[i] + outputv[i]) * gear_ratios[i];
    }
    pidp[2].setpoint = reference[2][0];
    pidp[2].measurement = estimate[2][0];
    pidv[2].setpoint = reference[2][1];
    pidv[2].measurement = estimate[2][1];
    if (reference[2][2] == 1) { // if state 10 is 1 then chassis heading is position controlled 
        pidp[2].K[0] = gains[6];
        pidp[2].K[1] = gains[7];
        pidp[2].K[2] = gains[8];
        pidv[2].K[0] = gains[9];
        pidv[2].K[1] = gains[10];
        pidv[2].K[2] = gains[11];
        pidv[2].K[3] = 0;
    } else {
        pidp[2].K[0] = 0;
        pidp[2].K[1] = 0;
        pidp[2].K[2] = 0;
        pidv[2].K[0] = 1;
        pidv[2].K[1] = 0;
        pidv[2].K[2] = 0;
        pidv[2].K[3] = 0;
    }
    outputp[2] = pidp[2].filter(dt, false, false);
    outputv[2] = pidv[2].filter(dt, false, false);
    output[2] = (outputp[2] + outputv[2]) * gear_ratios[2];

    float chassis_heading = estimate[2][0];

    // Convert to motor velocities
    motor_velocity[1] = output[0] * cos(chassis_heading) + output[1] * sin(chassis_heading) + output[2];
    motor_velocity[2] = output[0] * sin(chassis_heading) - output[1] * cos(chassis_heading) + output[2];
    motor_velocity[3] = -output[0] * cos(chassis_heading) - output[1] * sin(chassis_heading) + output[2];
    motor_velocity[0] = -output[0] * sin(chassis_heading) + output[1] * cos(chassis_heading) + output[2];

    // Serial.printf("1: %f, 2: %f, 3: %f, 4: %f\n", motor_velocity[0], motor_velocity[1], motor_velocity[2], motor_velocity[3]);

    // Power limiting
    float power_buffer = ref->ref_data.robot_power_heat.buffer_energy;
    float power_limit_ratio = 1.0;
    float power_buffer_limit_thresh = gains[15];
    float power_buffer_critical_thresh = gains[16];
    if (power_buffer < power_buffer_limit_thresh) {
        power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
    }
    // Serial.printf("current_hp: %f", ref);

    // Low level velocity controller
    for (int i = 0; i < 4; i++) {
        PIDFilter pid;
        pid.setpoint = motor_velocity[i];
        pid.measurement = micro_estimate[i][1];
        pid.K[0] = gains[12];
        pid.K[1] = gains[13];
        pid.K[2] = gains[14];
        outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
    }
}

void XDriveVelocityController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();
    // High level velocity controller
    for (int i = 0; i < 2; i++) {
        pidv[i].K[0] = gains[0];
        pidv[i].K[1] = gains[1];
        pidv[i].K[2] = gains[2];
        pidv[i].K[3] = gains[3];

        pidv[i].setpoint = reference[i][1];
        pidv[i].measurement = estimate[i][1];
        output[i] = pidv[i].filter(dt, false, false) * gear_ratios[i];
    }
    pidp[2].setpoint = reference[2][0];
    pidp[2].measurement = estimate[2][0];
    pidv[2].setpoint = reference[2][1];
    pidv[2].measurement = estimate[2][1];
    if (reference[2][2] == 1) { // if state [2][2] is 1 (We dont use the accel spot for anything) then chassis heading is position controlled 
        pidp[2].K[0] = gains[4];
        pidp[2].K[1] = gains[5];
        pidp[2].K[2] = gains[6];
        pidv[2].K[0] = gains[7];
        pidv[2].K[1] = gains[8];
        pidv[2].K[2] = gains[9];
        pidv[2].K[3] = 0;
    } else {
        pidp[2].K[0] = 0;
        pidp[2].K[1] = 0;
        pidp[2].K[2] = 0;
        pidv[2].K[0] = 1;
        pidv[2].K[1] = 0;
        pidv[2].K[2] = 0;
        pidv[2].K[3] = 0;
    }
    outputp[2] = pidp[2].filter(dt, false, false);
    outputv[2] = pidv[2].filter(dt, false, false);
    output[2] = (outputp[2] + outputv[2]) * gear_ratios[2];
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
    float power_buffer_limit_thresh = gains[13];
    float power_buffer_critical_thresh = gains[14];
    if (power_buffer < power_buffer_limit_thresh) {
        power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
    }

    // Low level velocity controller
    for (int i = 0; i < 4; i++) {
        PIDFilter pid;
        pid.setpoint = motor_velocity[i];
        pid.measurement = micro_estimate[i][1];
        pid.K[0] = gains[10];
        pid.K[1] = gains[11];
        pid.K[2] = gains[12];
        outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
        // Serial.printf("motor %d error: %f output: %f\n", i, -micro_estimate[i][1] + motor_velocity[i], outputs[i]);
    }
}

void YawController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();
    float output = 0.0;

    pidp.K[0] = gains[0];
    pidp.K[1] = gains[1];
    pidp.K[2] = gains[2];
    pidv.K[0] = gains[3];
    pidv.K[1] = gains[4];
    pidv.K[2] = gains[5];

    pidp.setpoint = reference[3][0];
    pidp.measurement = estimate[3][0];

    pidv.setpoint = reference[3][1];
    pidv.measurement = estimate[3][1];

    output += pidp.filter(dt, true, true); // position wraps
    output += pidv.filter(dt, true, false); // no wrap for velocity
    output = constrain(output, -1.0, 1.0);
    outputs[0] = -output;
    outputs[1] = -output;

    // Serial.printf("Yaw est: %f, yaw ref: %f, yaw output: %f\n", estimate[3][0], reference[3][0], output);
}

void PitchController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();
    float output = 0.0;

    pidp.K[0] = gains[0];
    pidp.K[1] = gains[1];
    pidp.K[2] = gains[2];
    pidp.K[3] = gains[3] * sin(reference[4][0]);
    pidv.K[0] = gains[4];
    pidv.K[1] = gains[5];
    pidv.K[2] = gains[6];
    // Serial.printf("pitch angle: %f\n", estimate[4][0]);
    pidp.setpoint = reference[4][0];
    pidp.measurement = estimate[4][0];

    pidv.setpoint = reference[4][1];
    pidv.measurement = estimate[4][1];

    output += pidp.filter(dt, true, true); // position wraps
    output += pidv.filter(dt, true, false); // no wrap for velocity
    output = constrain(output, -1.0, 1.0);

    outputs[0] = output * gear_ratios[0];
    outputs[1] = output * gear_ratios[1];

    // Serial.printf("Pitch est: %f, pitch ref: %f, pitch output: %f\n", estimate[4][0], reference[4][0], output);
}

void FlywheelController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();

    pid_high.K[0] = gains[0];
    pid_high.K[1] = gains[1];
    pid_high.K[2] = gains[2];
    pid_high.K[3] = reference[5][1];

    pid_high.setpoint = reference[5][1];
    pid_high.measurement = estimate[5][1];
    float target_motor_velocity = pid_high.filter(dt, false, false) * gear_ratios[0];
    for (int i = 0; i < 2; i++) {
        pid_low.K[0] = gains[4];
        pid_low.K[1] = gains[5];
        pid_low.K[2] = gains[6];
        pid_low.K[3] = 0;

        pid_low.setpoint = -target_motor_velocity;
        if (i == 1) pid_low.setpoint = target_motor_velocity;

        pid_low.measurement = micro_estimate[i + 10][1];
        outputs[i] = pid_low.filter(dt, true, false);
    }
}

void FeederController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();
    
    pid_high.K[0] = gains[0];
    pid_high.K[1] = gains[1];
    pid_high.K[2] = gains[2];
    pid_high.K[3] = reference[6][1];

    pid_high.setpoint = reference[6][1];
    pid_high.measurement = estimate[6][1];
    float output = pid_high.filter(dt, false, false)*gear_ratios[0];
    pid_low.K[0] = gains[3];
    pid_low.K[1] = gains[4];
    pid_low.K[2] = gains[5];
    pid_low.K[3] = output * gear_ratios[1];

    pid_low.setpoint = output;
    pid_low.measurement = micro_estimate[12][1];
    output = pid_low.filter(dt, true, false);
    // Serial.printf("Feeder output: %f, ref: %f, FF: %f\n", output, pid_low.setpoint-micro_estimate[12][1], pid_low.K[3]);
    outputs[0] = output;
}

void SwitcherController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {
    float dt = timer.delta();

    pidp.K[0] = gains[0];
    pidp.K[1] = gains[1];
    pidp.K[2] = gains[2];
    pidp.K[3] = 0;

    pidv.K[0] = gains[4];
    pidv.K[1] = gains[5];
    pidv.K[2] = gains[6];
    // Serial.printf("Pushing into wall: %f, %f\n", estimate[0], reference[0]);
    // // Feed forward to push the switcher into the wall constantly with a small force
    if (estimate[7][0] > gains[7] && reference[7][0] > -gains[7] && !(reference[7][0] < gains[7])) {
        pidp.K[3] = gains[3];
        pidp.K[0] = 0;
        pidv.K[0] = 0;
    } else if (estimate[7][0] < -gains[7] && reference[7][0] < gains[7] && !(reference[7][0] > -gains[7])) {
        pidp.K[3] = -gains[3];
        pidp.K[0] = 0;
        pidv.K[0] = 0;
    } else {
        pidp.K[3] = 0;
    }

    // if(reference[0] > 0){
    //     pidp.K[3] = gains[3];
    // } else if(reference[0] < 0){
    //     pidp.K[3] = -gains[3];
    // } else {
    //     pidp.K[3] = 0;
    // }

    pidp.setpoint = reference[7][0]; // 1st index = position
    pidp.measurement = estimate[7][0];

    pidv.setpoint = reference[7][1];
    pidv.measurement = estimate[7][1];

    float outputp = pidp.filter(dt, true, false);
    float outputv = pidv.filter(dt, true, false);
    float output = outputp + outputv;

    outputs[0] = output;
}