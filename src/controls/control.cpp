#include "control.hpp"

void Control::init_controller(uint8_t can_id, uint8_t motor_id, int controller_type) {
    delete [] controllers;
    
    int index = ((can_id-1)*NUM_MOTORS_PER_BUS) + (motor_id-1);

    switch (controller_type) {
        case 0:
            controllers[index] = new NullController();
            break;
        case 1:
            controllers[index] = new PIDPositionController();
            break;
        case 2:
            controllers[index] = new PIDVelocityController();
            break;
        case 3:
            controllers[index] = new FullStateFeedbackController();
            break;
        default:
            controllers[index] = new NullController();
            break;
    }
}

void Control::step(float dt, float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float kinematics[NUM_MOTORS][STATE_LEN], float outputs[NUM_MOTORS]) {
    // Iterate through motors
    for (int m = 0; m < NUM_MOTORS; m++) {
        float output = 0;
        for (int j = 0; j < STATE_LEN; j++) {
            if (kinematics[m][j] == 0) continue;
            output += controllers[m]->step(dt,reference[j], estimate[j]) * kinematics[m][j];
        }
        this->output[m] = output;
    }
    memcpy(this->output, outputs, NUM_MOTORS * sizeof(float));
}