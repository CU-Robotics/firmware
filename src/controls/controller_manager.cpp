#include "controller_manager.hpp"

ControllerManager::ControllerManager()
{
}

void ControllerManager::init_controller(uint8_t can_id, uint8_t motor_id, int controller_type, float gains[NUM_GAINS])
{
    int index = ((can_id)*NUM_MOTORS_PER_BUS) + (motor_id - 1);

    switch (controller_type)
    {
    case 0:
        controllers[index] = new NullController();
        controllers[index]->set_gains(gains);
        break;
    case 1:
        controllers[index] = new PIDPositionController();
        controllers[index]->set_gains(gains);
        break;
    case 2:
        controllers[index] = new PIDVelocityController();
        controllers[index]->set_gains(gains);
        break;
    case 3:
        controllers[index] = new FullStateFeedbackController();
        controllers[index]->set_gains(gains);
        break;
    default:
        controllers[index] = new NullController();
        controllers[index]->set_gains(gains);
        break;
    }
}

void ControllerManager::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float kinematics_p[NUM_MOTORS][STATE_LEN], float kinematics_v[NUM_MOTORS][STATE_LEN], float outputs[NUM_MOTORS]) {
    // Iterate through motors
    for (int m = 0; m < NUM_MOTORS; m++) {
        float output = 0;
        for (int j = 0; j < STATE_LEN; j++) {
            if (kinematics_v[m][j] == 0 && kinematics_p[m][j] == 0) continue;
            float temp_reference[3];
            float temp_estimate[3];

            temp_reference[0] = reference[j][0] * kinematics_p[m][j];
            temp_estimate[0] = estimate[j][0] * kinematics_p[m][j];

            temp_reference[1] = reference[j][1] * kinematics_v[m][j];
            temp_estimate[1] = estimate[j][1] * kinematics_v[m][j];

            output += controllers[m]->step(temp_reference, temp_estimate);
        }
        outputs[m] = output;
    }
}

// void ControllerManager::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float outputs[NUM_MOTORS])
// {
//     for(int i = 0; i < NUM_CONTROLLERS; i++){
//         controllers[i]->step(reference, estimate, outputs);
//     }

// }