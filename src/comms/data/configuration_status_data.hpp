#pragma once

#include "comms/data/comms_data.hpp"
#include <stdint.h>

/// @brief Data struct for the configuration status of the robot. This is sent from firmware to indicate the status of the configuration process.
struct ConfigurationStatusData : Comms::CommsData{
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the ConfigurationStatusData struct.
    ConfigurationStatusData() : CommsData(Comms::TypeLabel::ConfigurationStatus, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(ConfigurationStatusData)) { } 
    /// @brief Whether the robot is ready for config packets to be sent. This is set to 1 when firmware receives the ConfigStart packet, and is set back to 0 when the entire configuration process is complete.
    uint32_t ready_for_config = 0;
    /// @brief Whether the robot is configured. This is set to 1 when the entire configuration process is complete.
    uint32_t is_configured = 0;
};