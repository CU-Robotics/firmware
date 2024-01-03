#include "control_manager.hpp"
#include 


control_manager::control_manager(rm_CAN* _can, float control_weights[WEIGHTS_LEN][STATE_LEN]) {
    can = _can;
    control_gains = control_weights;
}

void control_manager::step_motors() {
    // update the data that we have

    // runs the set control method on each motor
    for (int i = 0; i < NUM_MOTORS; i++) {

        // for can 1
        int ctrl_type = control_type[CAN_1][i];

        switch (ctrl_type) {
            case PID: {
                output[CAN_1][i] = step_PID(curr_state[CAN_1][i], set_state[CAN_1][i]);
            } break;
            // ... put the rest of the control methods for can 1
        }

        // for can 2
        ctrl_type = control_type[CAN_2][i];

        switch (ctrl_type) {
            case PID: {
                output[CAN_2][i] = step_PID(curr_state[CAN_2][i], set_state[CAN_1][i]);
            } break;
            // ... rest of control methods for can 2
        }

    }
}

//this needs to happen somewhere else
//
// void control_manager::update_motors() {
//     // update motor output 
//     step_motors();

//     // careful writing to motors here plz
//     for (int i = 0; i < NUM_MOTORS; i++) {
//         // can 1
//         can->write_motor(CAN_1, i, output[CAN_1][i]);
        
//         // can 2
//         can->write_motor(CAN_2, i, output[CAN_2][i]);
//     }
// }

void control_manager::step_PID_position(int row, std::vector<float> reference, std::vector<float> estimate) {
    

    float error = reference.get(0) - estimate.get(0);
    float KP = error * control_gains[row][0];

    float delta_error = error - control_data[row][0];
    float KD = delta_error * control_gains[row][2];

    float KI = control_data[row][1] * control_gains[row][1]; 

    control_data[row][0] = error;
    control_data[row][1] = control_data[row][1] + error;

}