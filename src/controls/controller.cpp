#include "controller.hpp"

Controller::Controller(float _controller_weights[STATE_LEN][WEIGHTS_LEN], int _controller_types[NUM_MOTORS], float _kinematics[NUM_MOTORS][STATE_LEN], int ctrl_weights_size, int ctrl_types_size, int kinematics_size) {
    std::memcpy(_controller_types, controller_types, ctrl_types_size);
    std::memcpy(_controller_weights, controller_weights, ctrl_weights_size);
    std::memcpy(_kinematics, kinematics, kinematics_size);
}

void Controller::step_controllers(float input[NUM_MOTORS]) {
    State::get_instance()->get_reference(reference);
    State::get_instance()->get_estimate(estimate);

    Configuration::get_instance()->get_controller_types(controller_types);
    Configuration::get_instance()->get_kinematics(kinematics);
    Configuration::get_instance()->get_controller_weights(controller_weights);

    //can1
    for(int i = 0; i < CAN1_LEN; i++){
        for (int m = 0; m < NUM_MOTORS; m++) {
            int ctrl_type = controller_types[m];

            for(int k = 0; k < STATE_LEN; k++){
                if(kinematics[1][m][k] != 0.0){
                    switch (ctrl_type) {
                        case 0: {
                            reference[k][0] *= kinematics[1][m][k];
                            reference[k][1] *= kinematics[1][m][k];
                            reference[k][2] *= kinematics[1][m][k];
                            outputs[m] = step_PID_position(m, reference[k], estimate[k]);
                        } break;
                    }
                }
            }
        }
    }

    //can2
    for(int i = 0; i < CAN2_LEN; i++){
        for (int m = 0; m < NUM_MOTORS; m++) {
            int ctrl_type = controller_types[m];

            for(int k = 0; k < STATE_LEN; k++){
                if(kinematics[2][m][k] != 0.0){
                    switch (ctrl_type) {
                        case 0: {
                            reference[k][0] *= kinematics[2][m][k];
                            reference[k][1] *= kinematics[2][m][k];
                            reference[k][2] *= kinematics[2][m][k];
                            outputs[m] = step_PID_position(m, reference[k], estimate[k]);
                        } break;
                    }
                }
            }
        }
    }
    std::memcpy(outputs, input, sizeof(outputs));
}

float Controller::step_PID_position(int motor_id, float reference[3], float estimate[3]) {
    float error = reference[0] - estimate[0];
    float delta_error = error - control_data[motor_id][0];

    if(delta_error == error) delta_error = 0; //make sure we don't have a super large starting KD because last error is 0
    
    float KP = error * controller_weights[motor_id][0];
    float KI = control_data[motor_id][1] * controller_weights[motor_id][1]; 
    float KD = delta_error * controller_weights[motor_id][2];
    
    control_data[motor_id][0] = error;
    control_data[motor_id][1] = control_data[row][1] + error;

    return KP+KI+KD;
}

float Controller::step_PID_velocity(int motor_id, float reference[3], float estimate[3]) {
    float error = reference[1] - estimate[1];
    float delta_error = error - control_data[motor_id][0];

    if(delta_error == error) delta_error = 0; //make sure we don't have a super large starting KD because last error is 0
    
    float KP = error * controller_weights[motor_id][4];
    float KI = control_data[motor_id][1] * controller_weights[motor_id][1]; 
    float KD = delta_error * controller_weights[motor_id][2];

    control_data[motor_id][0] = error;
    control_data[motor_id][1] = control_data[motor_id][1] + error;

    return KP+KI+KD;
}