#include "controller_manager.hpp"

void ControllerManager::init(const std::vector<NewConfig::Controller>& controller_configurations) {
    for(NewConfig::Controller controller_config : controller_configurations) {
        init_controller(controller_config);
    }
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
    for(Controller controller : controllers) {
        float outputs[CAN_MAX_MOTORS];
        controller.step(macro_reference, macro_estimate, micro_estimate, outputs);

        for(NewConfig::MotorName motor_name : controller.config().motor_names) {
            if(motor_name == NewConfig::MotorName::UnsetMotorName) break;
            can.write_motor_torque_by_name(motor_name, outputs[static_cast<uint32_t>(motor_name)]);
        }
    }
}