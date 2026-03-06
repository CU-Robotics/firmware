#pragma once

#include <cstdint>
#include <vector>      // for std::vector
#include "comms/config_data/controller.hpp" // for Controller
#include "comms/config_data/estimator.hpp"  // for LowLevelEstimator, HighLevelEstimator
#include "comms/config_data/motor.hpp"      // for Motor
#include "comms/config_data/sensor.hpp"     // for BuffEncoder, IcmImu, D200Lidar, RealsenseCamera
#include "comms/config_data/state.hpp"      // for StateConfig

namespace Cfg {

enum RobotId {
    UnsetRobotId,
    Hero,
    Engineer,
    Standard3,
    Standard4,
    Standard5,
    Aerial,
    Sentry,
    Dart,
    Radar,
    Outpost,
    Base,
};

struct ConfigStart : Comms::CommsData {
    uint32_t robot;
    uint32_t num_config_sections;

    ConfigStart() : Comms::CommsData(Comms::TypeLabel::ConfigStart, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(ConfigStart)) {
        robot = UnsetRobotId;
        num_config_sections = 0;
    }
};

struct RobotConfig {
    ConfigStart config_start;
    std::vector<Controller> controllers;
    std::vector<BuffEncoder> buff_encoders;
    std::vector<IcmImu> icm_imus;
    std::vector<D200Lidar> d200_lidars;
    std::vector<Estimator> estimators;
    std::vector<State> states;
    std::vector<Motor> motors;
    uint32_t num_sections_received = 0;

    bool is_configured() const {
        if (num_sections_received == config_start.num_config_sections && config_start.num_config_sections != 0) {
            return true;
        } else if(num_sections_received > config_start.num_config_sections) {
            // should never happen
            // assert(false && "Received more config sections than expected");
        }
        return false;
    }
};

}