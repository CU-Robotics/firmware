#include "control_manager.hpp"

control_manager::control_manager(rm_CAN* _can) {
    can = _can;
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

void control_manager::update_motors() {
    // update motor output
    step_motors();

    // careful writing to motors here plz
    for (int i = 0; i < NUM_MOTORS; i++) {
        // can 1
        can->write_motor(CAN_1, i, output[CAN_1][i]);
        
        // can 2
        can->write_motor(CAN_2, i, output[CAN_2][i]);
    }
}

void control_manager::step_PID(float curr, float set) {
    
}