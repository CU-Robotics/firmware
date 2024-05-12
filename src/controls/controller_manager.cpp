#include "controller_manager.hpp"

ControllerManager::ControllerManager() {}

void ControllerManager::init_controller(uint8_t can_id, uint8_t motor_id, int controller_type, int controller_level, float gains[NUM_GAINS]) {
    int index = ((can_id)*NUM_MOTORS_PER_BUS) + (motor_id - 1);

    switch (controller_type) {
    case 0:
        controllers[index][controller_level] = new NullController();
        controllers[index][controller_level]->set_gains(gains);
        break;
    case 1:
        controllers[index][controller_level] = new PIDPositionController(controller_level);
        controllers[index][controller_level]->set_gains(gains);
        break;
    case 2:
        controllers[index][controller_level] = new PIDVelocityController(controller_level);
        controllers[index][controller_level]->set_gains(gains);
        break;
    case 3:
        controllers[index][controller_level] = new FullStateFeedbackController(controller_level);
        controllers[index][controller_level]->set_gains(gains);
        break;
    case 4:
        controllers[index][controller_level] = new ChassisPIDVelocityController(controller_level);
        controllers[index][controller_level]->set_gains(gains);
        break;
    case 5:
        controllers[index][controller_level] = new PIDFVelocityController(controller_level);
        controllers[index][controller_level]->set_gains(gains);
        break;
    case 6:
        controllers[index][controller_level] = new SwitcherController(controller_level);
        controllers[index][controller_level]->set_gains(gains);
        break;
    default:
        controllers[index][controller_level] = new NullController();
        controllers[index][controller_level]->set_gains(gains);
        break;
    }
}

void ControllerManager::step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float kinematics_p[NUM_MOTORS][STATE_LEN], float kinematics_v[NUM_MOTORS][STATE_LEN], float outputs[NUM_MOTORS]) {
    // clear the outputs array before updating
    for (int i = 0;i < NUM_MOTORS;i++) outputs[i] = 0;

    float micro_reference[STATE_LEN];
    // Iterate through controller level 0
    for (int m = 0; m < NUM_MOTORS; m++) {
        float output = 0;
        for (int j = 0; j < STATE_LEN; j++) {
            if (kinematics_v[m][j] == 0 && kinematics_p[m][j] == 0) continue;
            float temp_macro_reference[3];
            float temp_macro_estimate[3];

            temp_macro_reference[0] = macro_reference[j][0] * kinematics_p[m][j];
            temp_macro_estimate[0] = macro_estimate[j][0] * kinematics_p[m][j];

            temp_macro_reference[1] = macro_reference[j][1] * kinematics_v[m][j];
            temp_macro_estimate[1] = macro_estimate[j][1] * kinematics_v[m][j];
            //itterate the high level controllers first
            output += controllers[m][0]->step(temp_macro_reference, temp_macro_estimate);
        }
        micro_reference[m] = output;
    }


    // Iterate through controller level 1
    for (int m = 0; m < NUM_MOTORS; m++) {
        float temp_micro_reference;
        float temp_micro_estimate[MICRO_STATE_LEN];
        temp_micro_reference = micro_reference[m];
        for (int j = 0; j < MICRO_STATE_LEN; j++) temp_micro_estimate[j] = micro_estimate[m][j];

        //itterate the low level controllers second
        outputs[m] += controllers[m][1]->step(temp_micro_reference, temp_micro_estimate);
    }

    // Iterate through controller level 2
    for (int m = 0; m < NUM_MOTORS; m++) {
        float output = 0;
        for (int j = 0; j < STATE_LEN; j++) {
            if (kinematics_v[m][j] == 0 && kinematics_p[m][j] == 0) continue;
            float temp_macro_reference[3];
            float temp_macro_estimate[3];

            temp_macro_reference[0] = macro_reference[j][0] * kinematics_p[m][j];
            temp_macro_estimate[0] = macro_estimate[j][0] * kinematics_p[m][j];

            temp_macro_reference[1] = macro_reference[j][1] * kinematics_v[m][j];
            temp_macro_estimate[1] = macro_estimate[j][1] * kinematics_v[m][j];

            //itterate the overarching controllers last
            output += controllers[m][2]->step(temp_macro_reference, temp_macro_estimate);
        }
        outputs[m] += output;
    }
}

// void ControllerManager::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float outputs[NUM_MOTORS])
// {
//     for(int i = 0; i < NUM_CONTROLLERS; i++){
//         controllers[i]->step(reference, estimate, outputs);
//     }

// }