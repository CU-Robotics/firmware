#include "controller_manager.hpp"

void ControllerManager::init(CANManager* _can, const Config* config) {
    can = _can;
    config_data = config;
    // intializes all controllers given the controller_types matrix from the config
    for (int i = 0; i < NUM_ROBOT_CONTROLLERS; i++) {
        init_controller(config_data->controller_info[i][0], config_data->gains[i], config_data->gear_ratios[i]);
    }
    Serial.printf("num controllers: %d\n", num_controllers);
}

void ControllerManager::init_controller(int controller_type, const float gains[NUM_GAINS], const float gear_ratios[CAN_MAX_MOTORS]) {
    // intializes controller based on type defined in config yaml
    switch (controller_type) {
    case 0:
        if (controllers[num_controllers] == nullptr) controllers[num_controllers] = new NullController();
        break;
    case 1:
        controllers[num_controllers] = new XDrivePositionController();
        break;
    case 2:
        controllers[num_controllers] = new XDriveVelocityController();
        break;
    case 3:
        controllers[num_controllers] = new YawController();
        break;
    case 4:
        controllers[num_controllers] = new PitchController();
        break;
    case 5:
        controllers[num_controllers] = new FlywheelController();
        break;
    case 6:
        controllers[num_controllers] = new FeederController();
        break;
    case 7:
        controllers[num_controllers] = new SwitcherController();
        break;
    default:
        if (controllers[num_controllers] == nullptr) controllers[num_controllers] = new NullController();
        break;
    }
    controllers[num_controllers]->set_gains(gains);
    controllers[num_controllers]->set_gear_ratios(gear_ratios);
    if (controller_type != 0) num_controllers++;
}

void ControllerManager::step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN]) {
    float outputs[CAN_MAX_MOTORS];
    // iterate through all the created controllers
    for (int i = 0; i < num_controllers; i++) {
        // grab the motor outputs for this controller
        controllers[i]->step(macro_reference, macro_estimate, micro_estimate, outputs);

        Serial.printf("Controller %d\n", i);
        
        // iterate through all the motors this controller sets
        for (size_t j = 0; j < CAN_MAX_MOTORS + 1; j++) {
            if (config_data->controller_info[i][j + 1] < 0) break;
            Serial.printf("\tMotor %d: %f\n", config_data->controller_info[i][j + 1], outputs[j]);
            actuator_write(config_data->controller_info[i][j + 1], outputs[j]);
        }
    }
}

// motor_info[Global ID][type, Physical ID, Physical Bus]
// motor_info[id][0] -- type
// motor_info[id][1] -- Phys ID
// motor_info[id][2] -- Phys Bus

// motor_info:
//  - [type, phys_id, phys_bus]
//  - [type, phys_id, phys_bus]
//  - [type, phys_id, phys_bus]

void ControllerManager::actuator_write(int motor_id, float value) {
    // switch((int) config_data->motor_info[motor_id][MOTOR_INFO_TYPE]) {
    // case C610: // 0
    //     can->write_motor_norm((uint16_t) config_data->motor_info[motor_id][MOTOR_INFO_BUS], (uint16_t) config_data->motor_info[motor_id][MOTOR_INFO_ID], C610, value);
    //     break;
    // case C620: // 1
    //     can->write_motor_norm((uint16_t) config_data->motor_info[motor_id][MOTOR_INFO_BUS], (uint16_t) config_data->motor_info[motor_id][MOTOR_INFO_ID], C620, value);
    //     break;
    // default:
    //     break;
    // }

    can->write_motor_torque((uint32_t)config_data->motor_info[motor_id][MOTOR_INFO_BUS], (uint32_t)config_data->motor_info[motor_id][MOTOR_INFO_ID], value);
}
