#pragma once

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"
#include "controls/state.hpp"

/// @brief Structure to send stereo cam trigger data to comms.
struct StereoCamTriggerData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the StereoCamTriggerData struct.
    StereoCamTriggerData() : CommsData(Comms::TypeLabel::StereoCamTriggerData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(StereoCamTriggerData)) { }
    /// @brief Constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the StereoCamTriggerData struct, and also sets the camera trigger name.
    /// @param name The name of the camera trigger that this data corresponds to.
    StereoCamTriggerData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::StereoCamTriggerData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(StereoCamTriggerData)), camera_trigger_name(name) { }
    /// @brief The name of the camera trigger that this data corresponds to.
    Cfg::SensorName camera_trigger_name = Cfg::SensorName::UnsetSensorName;
    /// @brief the state of the robot at the time of stereo trigger
    State::Raw state[static_cast<size_t>(Cfg::StateName::StateNameCount)] = { {0, 0, 0} };

    /// @brief Print the stereo camera trigger data to the serial console for debugging purposes.
    void print() const {
        printf("StereoCamTriggerData - camera_trigger_name: %lu\n", static_cast<unsigned long>(camera_trigger_name));
    }
};

/// @brief Comms Packet used to reset the stereo trigger counters, should be renamed
struct StartStereoTrigger : Comms::CommsData {
    StartStereoTrigger() : CommsData(Comms::TypeLabel::StartStereoTrigger, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(StartStereoTrigger)) { }
    /// @brief the name of the stereo camera trigger
    Cfg::SensorName camera_trigger_name = Cfg::SensorName::UnsetSensorName;
};

/// @brief Comms Packet that is currently unused
struct StopStereoTrigger : Comms::CommsData {
    StopStereoTrigger() : CommsData(Comms::TypeLabel::StopStereoTrigger, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(StopStereoTrigger)) { }
    /// @brief the name of the stereo camera trigger
    Cfg::SensorName camera_trigger_name = Cfg::SensorName::UnsetSensorName;
};

/// @brief used to keep track of the stereo cam state changes received from hive
struct StereoCamStartStop {
    /// @brief if the stereo camera is being triggered
    bool running = false;
    /// @brief if a start request was received in the last packet
    bool start_received = false;
    /// @brief if a stop request was recieved in the last packet
    bool stop_received = false;
};
