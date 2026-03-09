#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "sensors/transmitter/transmitter_utils.hpp"
#include <stdint.h>                             // uintN_t

/// @brief Structure for the Transmitter
struct ET16SData : Comms::CommsData {
    ET16SData() : CommsData(Comms::TypeLabel::ET16SData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(ET16SData)) { }
	SwitchPos safety_switch = SwitchPos::INVALID;
    float r_stick_x;
	float r_stick_y;
	float l_stick_x;
	float l_stick_y;
	SwitchPos switch_b = SwitchPos::INVALID;
	SwitchPos switch_c = SwitchPos::INVALID;
	SwitchPos switch_d = SwitchPos::INVALID;
	SwitchPos switch_e = SwitchPos::INVALID;
	SwitchPos switch_f = SwitchPos::INVALID;
	SwitchPos switch_g = SwitchPos::INVALID;
	SwitchPos switch_h = SwitchPos::INVALID;
	float l_slider = 0;
	float r_slider = 0;
	float trim_one = 0;
	float trim_two = 0;
	float trim_three = 0;
	float trim_four = 0;
	float trim_five = 0;
	float trim_six = 0;
	float l_dial = 0;
	float r_dial = 0;
};