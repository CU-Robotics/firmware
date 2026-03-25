#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "sensors/transmitter/transmitter_utils.hpp"
#include <stdint.h>                             // uintN_t

/// @brief data struct for sending ET16S transmitter data to comms
struct ET16SData : Comms::CommsData {
	/// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the ET16SData struct.
    ET16SData() : CommsData(Comms::TypeLabel::ET16SData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(ET16SData)) { }
	/// @brief Safety switch position
	SwitchPos safety_switch = SwitchPos::INVALID;
	/// @brief r_stick_x value from -1 to 1
    float r_stick_x = 0.0;
	/// @brief r_stick_y value from -1 to 1
	float r_stick_y = 0.0;
	/// @brief l_stick_x value from -1 to 1
	float l_stick_x = 0.0;
	/// @brief l_stick_y value from -1 to 1
	float l_stick_y = 0.0;
	/// @brief switch_b position
	SwitchPos switch_b = SwitchPos::INVALID;
	/// @brief switch_c position
	SwitchPos switch_c = SwitchPos::INVALID;
	/// @brief switch_d position
	SwitchPos switch_d = SwitchPos::INVALID;
	/// @brief switch_e position
	SwitchPos switch_e = SwitchPos::INVALID;
	/// @brief switch_f position
	SwitchPos switch_f = SwitchPos::INVALID;
	/// @brief switch_g position
	SwitchPos switch_g = SwitchPos::INVALID;
	/// @brief switch_h position
	SwitchPos switch_h = SwitchPos::INVALID;
	/// @brief left slider value
	float l_slider = 0.0;
	/// @brief right slider value
	float r_slider = 0.0;
	/// @brief trim one value
	float trim_one = 0.0;
	/// @brief trim two value
	float trim_two = 0.0;
	/// @brief trim three value
	float trim_three = 0.0;
	/// @brief trim four value
	float trim_four = 0.0;
	/// @brief trim five value
	float trim_five = 0.0;
	/// @brief trim six value
	float trim_six = 0.0;
	/// @brief left dial value
	float l_dial = 0.0;
	/// @brief right dial value
	float r_dial = 0.0;
};