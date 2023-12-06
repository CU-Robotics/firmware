#include "control_manager.hpp"

control_manager::control_manager() {
    can.init();
}


void control_manager::step_motors() {
    // runs the set control method on each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        // for can 1
        int ctrl_type = control_type[CAN_1][i];

        switch (ctrl_type) {
            case PID: {
                output[CAN_1][i] = step_PID(curr_state[CAN_1][i], set_state[CAN_1][i]);
            } break;
            // put the rest of the control methods here
        }

    }
}

void control_manager::update_motors() {
    // write to motors

    /*
        WARNING: Safety code has not yet been
        implemented for these write functions.
        DO NOT upload this method to the teensy.
    */
    for (int i = 0; i < NUM_MOTORS; i++) {
        // can 1
        can.write_motor(CAN_1, i, output[CAN_1][i]);
        
        // can 2
        can.write_motor(CAN_2, i, output[CAN_2][i]);
    }

    // write to motors
    can.write();
}