#include "controller_manager.hpp"

void ControllerManager::init(CANManager* _can, const std::vector<NewConfig::Controller>& controller_configs) {
    can = _can;

    for(NewConfig::Controller controller_config : controller_configs) {
        init_controller(controller_config.controller_type, controller_config.gains, controller_config.gear_ratios);
    }

    // intializes all controllers given the controller_types matrix from the config
    for (int i = 0; i < NUM_ROBOT_CONTROLLERS; i++) {
        init_controller(config_data->controller_info[i][0], config_data->gains[i], config_data->gear_ratios[i]);
    }
    Serial.printf("num controllers: %d\n", num_controllers);
}

void ControllerManager::init_controller(NewConfig::Controller controller_config) {

    switch (controller_config.controller_type) {
        case NewConfig::UnsetControllerType:
            controllers.push_back(NullController(controller_config));
            break;
        case NewConfig::XDrivePositionController:
            controllers.push_back(XDrivePositionController(controller_config));
            break;
        case NewConfig::XDriveVelocityController:
            controllers.push_back(XDriveVelocityController(controller_config));
            break;
        case NewConfig::YawController:
            controllers.push_back(YawController(controller_config));
            break;
        case NewConfig::PitchController:
            controllers.push_back(PitchController(controller_config));
            break;
        case NewConfig::FlywheelController:
            controllers.push_back(FlywheelController(controller_config));
            break;
        case NewConfig::FeederController:
            controllers.push_back(FeederController(controller_config));
            break;
        default:
            Serial.printf("ControllerManager::init_controller: Unrecognized controller type %d\n", controller_config.controller_type);
            break;
    }
}

void ControllerManager::step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN]) {
    float outputs[CAN_MAX_MOTORS];
    // iterate through all the created controllers
    for (int i = 0; i < num_controllers; i++) {
        // grab the motor outputs for this controller
        controllers[i]->step(macro_reference, macro_estimate, micro_estimate, outputs);
        
        // iterate through all the motors this controller sets
        for (size_t j = 0; j < CAN_MAX_MOTORS + 1; j++) {
            if (config_data->controller_info[i][j + 1] < 0) break;
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
