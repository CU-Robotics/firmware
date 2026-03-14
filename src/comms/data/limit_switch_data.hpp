#pragma once

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

/// @brief Limit switch data for comms
struct LimitSwitchData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the LimitSwitchData struct.
    LimitSwitchData() : CommsData(Comms::TypeLabel::LimitSwitchData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LimitSwitchData)) { }
    /// @brief Constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the LimitSwitchData struct, and also sets the switch name.
    /// @param name The name of the limit switch that this data corresponds to.
    LimitSwitchData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::LimitSwitchData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LimitSwitchData)), switch_name(name) { }
    /// @brief The name of the limit switch that this data corresponds to.
    Cfg::SensorName switch_name;
    /// @brief Whether the limit switch is currently pressed. 1 for pressed, 0 for not pressed.
    uint8_t is_pressed;
    /// @brief Print the limit switch data to the serial console for debugging purposes.
    void print() const {
        printf("LimitSwitchData - switch_name: %lu, is_pressed: %u\n", static_cast<uint32_t>(switch_name), is_pressed);
    }
};