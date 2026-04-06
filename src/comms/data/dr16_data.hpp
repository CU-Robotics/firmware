#pragma once
#include "sensors/transmitter/transmitter_utils.hpp"
#include "comms/data/comms_data.hpp"
/// @brief data struct for sending DR16 transmitter data to comms
struct DR16Data : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the DR16Data struct.
    DR16Data() : CommsData(Comms::TypeLabel::DR16Data, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(DR16Data)) { }
    /// mouse x velocity
    int16_t mouse_x = 0;
    /// mouse y velocity
    int16_t mouse_y = 0;
    /// mouse z velocity
    int16_t mouse_z = 0;
    /// left mouse button status
    uint8_t l_mouse_button = 0;
    /// right mouse button status
    uint8_t r_mouse_button = 0;
    /// left switch status
    SwitchPos l_switch = SwitchPos::INVALID;
    /// right switch status
    SwitchPos r_switch = SwitchPos::INVALID;
    /// left stick x axis
    float l_stick_x = 0;
    /// left stick y axis
    float l_stick_y = 0;
    /// right stick x axis
    float r_stick_x = 0;
    /// right stick y axis
    float r_stick_y = 0;
    /// wheel
    float wheel = 0;
    /// @brief the state of all the keys
    Keys keys;
};