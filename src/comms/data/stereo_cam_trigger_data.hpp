#pragma once

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

/// @brief Structure to send stereo cam trigger data to comms.
struct StereoCamTriggerData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the StereoCamTriggerData struct.
    StereoCamTriggerData() : CommsData(Comms::TypeLabel::StereoCamTriggerData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(StereoCamTriggerData)) { }
    /// @brief Constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the StereoCamTriggerData struct, and also sets the camera trigger name.
    /// @param name The name of the camera trigger that this data corresponds to.
    StereoCamTriggerData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::StereoCamTriggerData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(StereoCamTriggerData)), camera_trigger_name(name) { }
    /// @brief The name of the camera trigger that this data corresponds to.
    Cfg::SensorName camera_trigger_name = Cfg::SensorName::UnsetSensorName;

    /// State matching has not been designed yet.

    /// @brief Print the stereo camera trigger data to the serial console for debugging purposes.
    void print() const {
        printf("StereoCamTriggerData - camera_trigger_name: %lu\n", static_cast<uint32_t>(camera_trigger_name));
    }
};