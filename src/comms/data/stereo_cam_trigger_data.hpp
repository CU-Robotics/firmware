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

    State::Raw state[static_cast<size_t>(Cfg::StateName::StateNameCount)] = { {0, 0, 0} };

    /// @brief Print the stereo camera trigger data to the serial console for debugging purposes.
    void print() const {
        printf("StereoCamTriggerData - camera_trigger_name: %lu\n", static_cast<unsigned long>(camera_trigger_name));
    }
};

struct StartStereoTrigger : Comms::CommsData {
    StartStereoTrigger() : CommsData(Comms::TypeLabel::StartStereoTrigger, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(StartStereoTrigger)) { }
    Cfg::SensorName camera_trigger_name = Cfg::SensorName::UnsetSensorName;
};

struct StopStereoTrigger : Comms::CommsData {
    StopStereoTrigger() : CommsData(Comms::TypeLabel::StopStereoTrigger, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(StopStereoTrigger)) { }
    Cfg::SensorName camera_trigger_name = Cfg::SensorName::UnsetSensorName;
};

struct StereoCamStartStop {
    bool running = false;
    bool start_received = false;
    bool stop_received = false;
};