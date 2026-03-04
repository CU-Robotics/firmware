#include "controller_manager.hpp"

void ControllerManager::init(const std::vector<NewConfig::Controller>& controller_configurations) {
    for(const NewConfig::Controller& controller_config : controller_configurations) {
        init_controller(controller_config);
    }
}

void ControllerManager::init_controller(const NewConfig::Controller& controller_config) {
    switch (controller_config.controller_type) {
        case NewConfig::UnsetControllerType:
            Serial.println("ControllerManager::init_controller: Unset controller type, skipping");
            break;
        case NewConfig::XDrivePositionController:
            controllers.push_back(std::make_unique(XDrivePositionController(controller_config)));
            break;
        case NewConfig::XDriveVelocityController:
            controllers.push_back(std::make_unique(XDriveVelocityController(controller_config)));
            break;
        case NewConfig::YawController:
            controllers.push_back(std::make_unique(YawController(controller_config)));
            break;
        case NewConfig::PitchController:
            controllers.push_back(std::make_unique(PitchController(controller_config)));
            break;
        case NewConfig::FlywheelController:
            controllers.push_back(std::make_unique(FlywheelController(controller_config)));
            break;
        case NewConfig::FeederController:
            controllers.push_back(std::make_unique(FeederController(controller_config)));
            break;
        default:
            Serial.printf("ControllerManager::init_controller: Unrecognized controller type %d\n", controller_config.controller_type);
            break;
    }
}

void ControllerManager::step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN]) {
    for(std::unique_ptr<Controller>& controller : controllers) {

        float outputs[CAN_MAX_MOTORS];
        controller->step(macro_reference, macro_estimate, micro_estimate, outputs);

        for(NewConfig::MotorName motor_name : controller->config().motor_names) {
            // If we reached an unset motor, we have reached the end of the motor list for this controller, so break out of the loop
            if(motor_name == NewConfig::MotorName::UnsetMotorName) break;
            // Send controller output to the motor
            can.write_motor_torque_by_name(motor_name, outputs[static_cast<uint32_t>(motor_name)]);
        }
    }
}