#include "controller_manager.hpp"
#include "controller.hpp"

void ControllerManager::init(const std::vector<Cfg::Controller>& controller_configurations, CANManager& can) {
    available_motors.clear();
    for (int i = 0; i < (uint32_t)(Cfg::MotorName::MotorNameCount); i++) {
        available_motors.push_back(static_cast<Cfg::MotorName>(i));
    }
    for(const Cfg::Controller& controller_config : controller_configurations) {
        init_controller(controller_config, can);
    }
}

void ControllerManager::init_controller(const Cfg::Controller& controller_config, CANManager& can) {
    switch (controller_config.controller_type) {
        case Cfg::ControllerType::UnsetControllerType:
            Serial.println("ControllerManager::init_controller: Unset controller type, skipping");
            break;
        case Cfg::ControllerType::XDrivePositionController:
            controllers.push_back(std::make_unique(XDriveController(controller_config, can, available_motors)));
            break;
        case Cfg::ControllerType::YawController:
            controllers.push_back(std::make_unique(YawController(controller_config, can, available_motors)));
            break;
        case Cfg::ControllerType::PitchController:
            controllers.push_back(std::make_unique(PitchController(controller_config, can, available_motors)));
            break;
        case Cfg::ControllerType::FlywheelController:
            controllers.push_back(std::make_unique(FlywheelController(controller_config, can, available_motors)));
            break;
        case Cfg::ControllerType::FeederController:
            controllers.push_back(std::make_unique(FeederController(controller_config, can, available_motors)));
            break;
        default:
            Serial.printf("ControllerManager::init_controller: Unrecognized controller type %d\n", controller_config.controller_type);
            break;
    }
}