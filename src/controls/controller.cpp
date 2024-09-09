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

        pidv[i].K[0] = gains[4];
        pidv[i].K[1] = gains[5];
        pidv[i].K[2] = gains[6];

        outputp[i] = pidp[i].filter(dt, false, false);
        outputv[i] = pidv[i].filter(dt, false, false);
        output[i] = outputp[i] + outputv[i];
    }
    pidp[2].setpoint = reference[2][0];
    pidp[2].measurement = estimate[2][0];

    pidp[2].K[0] = gains[0];
    pidp[2].K[1] = gains[1];
    pidp[2].K[2] = gains[2];

    pidv[2].setpoint = reference[2][1]; 
    pidv[2].measurement = estimate[2][1];

    pidv[2].K[0] = gains[4];
    pidv[2].K[1] = gains[5];
    pidv[2].K[2] = gains[6];

    outputp[2] = pidp[i].filter(dt, false, false);
    outputv[2] = pidv[i].filter(dt, false, false);
    output[2] = outputp[2] + outputv[2];

    // Convert to motor velocities
    motor_velocity[0] = output[0] + output[1] + output[2];
    motor_velocity[1] = output[0] - output[1] + output[2];
    motor_velocity[2] = -output[0] - output[1] + output[2];
    motor_velocity[3] = -output[0] + output[1] + output[2];

    // Low level velocity controller


    return output;
}

XDriveVelocityController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
        float dt = timer.delta();
        pid.setpoint = reference; // 1st index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];
        float power_buffer = ref.ref_data.robot_power_heat.buffer_energy;
        
        // Power limiting
        
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = gains[3];
        float power_buffer_critical_thresh = gains[4];
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
        }
        float output = pid.filter(dt, true, false) * power_limit_ratio;

        return output;
}

SwitcherController::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]){
    float dt = timer.delta();
    pid.setpoint = reference; // 1st index = position
    pid.measurement = estimate[0];
    pid.K[0] = gains[0];
    pid.K[1] = gains[1];
    pid.K[2] = gains[2];
    float power_buffer = ref.ref_data.robot_power_heat.buffer_energy;

    // Power limiting

    float power_limit_ratio = 1.0;
    float power_buffer_limit_thresh = gains[3];
    float power_buffer_critical_thresh = gains[4];
    if (power_buffer < power_buffer_limit_thresh) {
        power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
    }
    float output = pid.filter(dt, true, false) * power_limit_ratio;
    
    return output;
}