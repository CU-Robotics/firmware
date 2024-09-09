#include "controller.hpp"

XDrivePositionController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
    float dt = timer.delta();
    // High level position controller
    for(int i = 0; i < 2; i++){
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
        output[i] = (outputp[i] + outputv[i])*gear_ratios[i];
    }
    pidp[2].setpoint = reference[2][0];
    pidp[2].measurement = estimate[2][0];
    pidv[2].setpoint = reference[2][1]; 
    pidv[2].measurement = estimate[2][1];
    if(reference[10][0] == 1){ // if state 10 is 1 then chassis heading is position controlled 
        pidp[2].K[0] = gains[6];
        pidp[2].K[1] = gains[7];
        pidp[2].K[2] = gains[8];
        pidv[2].K[0] = gains[9];
        pidv[2].K[1] = gains[10];
        pidv[2].K[2] = gains[11];
        pidv[2].K[3] = 0;
    }else{
        pidp[2].K[0] = 0;
        pidp[2].K[1] = 0;
        pidp[2].K[2] = 0;
        pidv[2].K[0] = 0;
        pidv[2].K[1] = 0;
        pidv[2].K[2] = 0;
        pidv[2].K[3] = 1;
    }
    outputp[2] = pidp[i].filter(dt, false, false);
    outputv[2] = pidv[i].filter(dt, false, false);
    output[2] = (outputp[2] + outputv[2])*gear_ratios[2];

    // Adjust for chassis heading so control is field relative
    output[0] = output[0]*sin(reference[2][0]) + output[1]*cos(reference[2][0]);
    output[1] = output[1]*cos(reference[2][0]) - output[0]*sin(reference[2][0]);
    // Convert to motor velocities
    motor_velocity[0] = output[0] + output[1] + output[2];
    motor_velocity[1] = output[0] - output[1] + output[2];
    motor_velocity[2] = -output[0] - output[1] + output[2];
    motor_velocity[3] = -output[0] + output[1] + output[2];

    // Power limiting
    float power_buffer = ref.ref_data.robot_power_heat.buffer_energy;
    float power_limit_ratio = 1.0;
    float power_buffer_limit_thresh = gains[3];
    float power_buffer_critical_thresh = gains[4];
    if (power_buffer < power_buffer_limit_thresh) {
        power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
    }

    // Low level velocity controller
    for(int i = 0; i < 4; i++){
        PIDFilter pid;
        pid.setpoint = motor_velocity[i];
        pid.measurement = micro_estimate[i][1];
        pid.K[0] = gains[12];
        pid.K[1] = gains[13];
        pid.K[2] = gains[14];
        outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
    }
}

XDriveVelocityController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
    float dt = timer.delta();
    // High level velocity controller
    for(int i = 0; i < 2; i++){
        pidv[i].setpoint = reference[i][1]; 
        pidv[i].measurement = estimate[i][1];
        pidv[i].K[3] = 1;
        output[i] = pidv[i].filter(dt, false, false)*gear_ratios[i];
    }
    pidp[2].setpoint = reference[2][0];
    pidp[2].measurement = estimate[2][0];
    pidv[2].setpoint = reference[2][1]; 
    pidv[2].measurement = estimate[2][1];
    if(reference[10][0] == 1){ // if state 10 is 1 then chassis heading is position controlled 
        pidp[2].K[0] = gains[0];
        pidp[2].K[1] = gains[1];
        pidp[2].K[2] = gains[2];
        pidv[2].K[0] = gains[3];
        pidv[2].K[1] = gains[4];
        pidv[2].K[2] = gains[5];
        pidv[2].K[3] = 0;
    }else{
        pidp[2].K[0] = 0;
        pidp[2].K[1] = 0;
        pidp[2].K[2] = 0;
        pidv[2].K[0] = 0;
        pidv[2].K[1] = 0;
        pidv[2].K[2] = 0;
        pidv[2].K[3] = 1;
    }
    outputp[2] = pidp[i].filter(dt, false, false);
    outputv[2] = pidv[i].filter(dt, false, false);
    output[2] = (outputp[2] + outputv[2])*gear_ratios[2];

    // Adjust for chassis heading so control is field relative
    output[0] = output[0]*sin(reference[2][0]) + output[1]*cos(reference[2][0]);
    output[1] = output[1]*cos(reference[2][0]) - output[0]*sin(reference[2][0]);
    // Convert to motor velocities
    motor_velocity[0] = output[0] + output[1] + output[2];
    motor_velocity[1] = output[0] - output[1] + output[2];
    motor_velocity[2] = -output[0] - output[1] + output[2];
    motor_velocity[3] = -output[0] + output[1] + output[2];

    // Power limiting
    float power_buffer = ref.ref_data.robot_power_heat.buffer_energy;
    float power_limit_ratio = 1.0;
    float power_buffer_limit_thresh = gains[6];
    float power_buffer_critical_thresh = gains[7];
    if (power_buffer < power_buffer_limit_thresh) {
        power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
    }

    // Low level velocity controller
    for(int i = 0; i < 4; i++){
        PIDFilter pid;
        pid.setpoint = motor_velocity[i];
        pid.measurement = micro_estimate[i][1];
        pid.K[0] = gains[6];
        pid.K[1] = gains[7];
        pid.K[2] = gains[8];
        outputs[i] = pid.filter(dt, true, false) * power_limit_ratio;
    }
}

YawController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
    float dt = timer.delta();
    float output = 0.0;

    pidp.K[0] = gains[0];
    pidp.K[1] = gains[1];
    pidp.K[2] = gains[2];
    pidv.K[0] = gains[4];
    pidv.K[1] = gains[5];
    pidv.K[2] = gains[6];

    pidp.setpoint = reference[3][0];
    pidp.measurement = estimate[3][0];

    pidv.setpoint = reference[3][1];
    pidv.measurement = estimate[3][1];

    output += pidp.filter(dt, true, true); // position wraps
    output += pidv.filter(dt, true, false); // no wrap for velocity
    output = constrain(output, -1.0, 1.0);
    outputs[0] = output;
    outputs[1] = output;
}

PitchController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
    float dt = timer.delta();
    float output = 0.0;

    pid1.K[0] = gains[0];
    pid1.K[1] = gains[1];
    pid1.K[2] = gains[2];
    pid1.K[3] = gains[3] * sin(reference[4][0]);
    pid2.K[0] = gains[4];
    pid2.K[1] = gains[5];
    pid2.K[2] = gains[6];

    pid1.setpoint = reference[4][0];
    pid1.measurement = estimate[4][0];

    pid2.setpoint = reference[4][1];
    pid2.measurement = estimate[4][1];

    output += pid1.filter(dt, true, true); // position wraps
    output += pid2.filter(dt, true, false); // no wrap for velocity
    output = constrain(output, -1.0, 1.0);

    outputs[0] = output * gear_ratios[0];
    outputs[1] = output * gear_ratios[1];
}

FlywheelController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){

}

FeederController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
    
}

SwitcherController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
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
    if(estimate[0] > gains[7] && reference[0] > -gains[7] && !(reference[0] < gains[7])) {
        pidp.K[3] = gains[3];
        pidp.K[0] = 0;
        pidv.K[0] = 0;
    } else if(estimate[0] < -gains[7] && reference[0] < gains[7] && !(reference[0] > -gains[7])) {
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

    pidp.setpoint = reference[0]; // 1st index = position
    pidp.measurement = estimate[0];

    pidv.setpoint = reference[1]; 
    pidv.measurement = estimate[1];

    float outputp = pidp.filter(dt, true, false);
    float outputv = pidv.filter(dt, true, false);
    float output = outputp + outputv;

    outputs[0] = output;
}






    


