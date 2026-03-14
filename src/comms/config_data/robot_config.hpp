#pragma once

#include <cstdint>
#include <vector>      // for std::vector
#include "comms/config_data/controller.hpp" // for Controller
#include "comms/config_data/estimator.hpp"  // for LowLevelEstimator, HighLevelEstimator
#include "comms/config_data/motor.hpp"      // for Motor
#include "comms/config_data/sensor.hpp"     // for BuffEncoder, IcmImu, D200Lidar, RealsenseCamera
#include "comms/config_data/state.hpp"      // for StateConfig
#include "comms/config_data/transmitter.hpp" // for Transmitter

namespace Cfg {
/// @brief The specific robot that this config is for.
enum RobotID {
    UnsetRobotID,
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
/// @brief The `ConfigStart` struct is the first packet sent from comms to controls to indicate the start of the config transmission.
struct ConfigStart : Comms::CommsData {
    /// @brief What robot this config is for
    RobotID robot;
    /// @brief The total number of config sections that will be sent after this packet. This is used to determine when the entire config has been received.
    uint32_t num_config_sections;
    /// @brief Default constructor
    ConfigStart() : Comms::CommsData(Comms::TypeLabel::ConfigStart, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(ConfigStart)) {
        robot = UnsetRobotID;
        num_config_sections = 0;
    }
};

/// @brief Mega struct that contains all of the configuration data for the robot.
struct RobotConfig {
    /// @brief Received config start data
    ConfigStart config_start;
    /// @brief List of all received controller configs
    std::vector<Controller> controllers;
    /// @brief List of all received buff encoder configs
    std::vector<BuffEncoder> buff_encoders;
    /// @brief List of all received rev encoder configs
    std::vector<RevEncoder> rev_encoders;
    /// @brief List of all received icm imu configs
    std::vector<IcmImu> icm_imus;
    /// @brief List of all received lsm imu configs
    std::vector<LsmImu> lsm_imus;
    /// @brief List of all received d200 lidar configs
    std::vector<D200Lidar> d200_lidars;
    /// @brief List of all received limit switch configs
    std::vector<LimitSwitch> limit_switches;
    /// @brief List of all received stereo camera trigger configs
    std::vector<StereoCamTrigger> stereo_cam_triggers;
    /// @brief List of all received estimator configs
    std::vector<Estimator> estimators;
    /// @brief List of all received state configs
    std::vector<State> states;
    /// @brief List of all received motor configs
    std::vector<Motor> motors;
    /// @brief Receieved transmitter config.
    Transmitter transmitter;
    /// @brief The number of config sections recieved so far.
    uint32_t num_sections_received = 0;

    /// @brief Whether this config is fully configured or not.
    /// @return true if the number of config sections received matches the number of config sections expected according to the config start packet, false otherwise.
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