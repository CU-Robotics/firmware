#include "controller_manager.hpp"

void ControllerManager::init() {
    // intializes all controllers given the controller_types matrix from the config
    for (int i = 0; i < NUM_CONTROLLERS; i++) {
        init_controller(config.controller_types[i], config.gains[i], config.gear_ratios[i]);
    }
}

void ControllerManager::init_controller(int controller_type, float gains[NUM_GAINS], float gear_ratios[NUM_MOTORS]) {
    // intializes controller based on type defined in config yaml
    switch (controller_type) {
    case 0:
        controllers[num_controllers] = new NullController();
        break;
    case 1:
        controllers[num_controllers++] = new PIDPositionController(controller_level);
        break;
    case 2:
        controllers[num_controllers++] = new PIDVelocityController(controller_level);
        break;
    case 3:
        controllers[num_controllers++] = new FullStateFeedbackController(controller_level);
        break;
    case 4:
        controllers[num_controllers++] = new ChassisPIDVelocityController(controller_level);
        break;
    case 5:
        controllers[num_controllers++] = new PIDFVelocityController(controller_level);
        break;
    case 6:
        controllers[num_controllers++] = new SwitcherController(controller_level);
        break;
    case 7:
        controllers[num_controllers++] = new ChassisFullStateFeedbackController(controller_level);
        break;
    default:
        controllers[num_controllers] = new NullController();
        break;
    }
    controllers[num_controllers]->set_gains(gains);
    controllers[num_controllers]->set_gear_ratios(gear_ratios);
}

void ControllerManager::step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN]) {
    float outputs[NUM_MOTORS];
    // iterate through all the created controllers
    for(int i = 0; i < num_controllers; i++) {
        // grab the motor outputs for this controller
        outputs = controllers[i].step(macro_reference, macro_estimate, micro_estimate);
        // iterate through all the motors this controller sets
        for(int j = 0; j < NUM_MOTORS; j++) {
            if(motor_config.controller_motor_outputs[i][j] < 0) continue;
            actuator_write(motor_config.controller_motor_outputs[i][j], outputs[j]);
        }
    }
}

// motor_types[Global ID][type, Physical ID, Physical Bus]
// motor_types[id][0] -- type
// motor_types[id][1] -- Phys ID
// motor_types[id][2] -- Phys Bus

// motor_types:
//  - [type, phys_id, phys_bus]
//  - [type, phys_id, phys_bus]
//  - [type, phys_id, phys_bus]

#define MOTOR_TYPE_TYPE 0
#define MOTOR_TYPE_ID 1
#define MOTOR_TYPE_BUS 2

void actuator_write(int motor_id, float value){
    switch(config.motor_types[motor_id][MOTOR_TYPE_TYPE])
    case C610: // 0
        can.write_motor_norm(config.motor_info[motor_id][MOTOR_TYPE_BUS], config.motor_info[motor_id][MOTOR_TYPE_ID], C610, value);
        break;
    case C620: // 1
        can.write_motor_norm(config.motor_info[motor_id][MOTOR_TYPE_BUS], config.motor_info[motor_id][MOTOR_TYPE_ID], C620, value);
        break;
    default:
        break;
}
