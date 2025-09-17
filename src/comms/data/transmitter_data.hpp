#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t
#include <optional>                             // std::optional

/// @brief Switch Position Enum
enum class SwitchPos{
	INVALID = 0,
	FORWARD,
	BACKWARD,
	MIDDLE
};

/// @brief Structure for the Transmitter
struct TransmitterData : Comms::CommsData {
    TransmitterData() : CommsData(Comms::TypeLabel::TransmitterData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TransmitterData)) { }

    /// mouse x velocity
	std::optional<float> mouse_x = {};
    /// mouse y velocity
	std::optional<float> mouse_y = {};
    /// mouse z velocity
	std::optional<float> mouse_z = {};
    /// left mouse button status
	std::optional<bool> l_mouse_button = {};
    /// right mouse button status
	std::optional<bool> r_mouse_button = {};
    /// left switch status
	SwitchPos l_switch = SwitchPos::INVALID;
    /// right switch status
	SwitchPos r_switch = SwitchPos::INVALID;
    /// left stick x axis
	float  l_stick_x = 0;
    /// left stick y axis
    float l_stick_y = 0;
    /// right stick x axis
    float r_stick_x = 0;
    /// right stick y axis
    float r_stick_y = 0;
    /// wheel
	std::optional<float> wheel = {};
	/// safety switch
	std::optional<SwitchPos> safety_switch = {};
	/// switch b
	std::optional<SwitchPos> switch_b = {};
	/// switch c
	std::optional<SwitchPos> switch_c = {};
	/// switch d
	std::optional<SwitchPos> switch_d = {};
	/// switch e
	std::optional<SwitchPos> switch_e = {};
	/// switch f
	std::optional<SwitchPos> switch_f = {};
	/// switch g
	std::optional<SwitchPos> switch_g = {};
	/// switch h
	std::optional<SwitchPos> switch_h = {};
	/// left dial
	std::optional<float> l_dial = {};
	/// right dial
	std::optional<float> r_dial = {};
    /// left slider
    std::optional<float> l_slider = {};
    /// right slider
    std::optional<float> r_slider = {};
	/// trim one
	std::optional<float> trim_one = {};
	/// trim two
	std::optional<float> trim_two = {};
	/// trim three
	std::optional<float> trim_three = {};
	/// trim four
	std::optional<float> trim_four = {};
	/// trim five
	std::optional<float> trim_five = {};
	/// trim six
	std::optional<float> trim_six = {};
	
	/// @brief Keys
	union {
        uint16_t raw = 0;
        struct {
            bool w     : 1;
            bool s     : 1;
            bool a     : 1;
            bool d     : 1;
            bool shift : 1;
            bool ctrl  : 1;
            bool q     : 1;
            bool e     : 1;
            bool r     : 1;
            bool f     : 1;
            bool g     : 1;
            bool z     : 1;
            bool x     : 1;
            bool c     : 1;
            bool v     : 1;
            bool b     : 1;
        } key;
    } keys;	
};
