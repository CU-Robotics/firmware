/**
 * @file vn100.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines the user side of the VectorNav VN100 driver, specifically the module interface. This includes module instantiation,
 *	      task spawning, option parsing, and the main run loop.
 * @date 2025-07
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#pragma once

#include <Arduino.h>

#include "protocol/serial.hpp"
#include "protocol/params.hpp"

// poll at 1kHz
static constexpr uint32_t _read_interval_us = 1000; // read interval in microseconds

class VN100 
{
public:
	VN100();
	~VN100();

	void loop();

private:
	bool init();
	bool ramp_up_baudrate();

	static void binary_message_callback(vn::bin::const_bin_buf_ref_t buf, vn::msg::len_t length,
					    vn::bin::BinaryMessage &msg);

private:
	vn::Serial _uart; // serial interface to the VN100

	bool _initialized{false}; // true if the VN100 has been initialized

	vn::bin::BinaryMessage bin_msg_1; // binary message 1 config
	vn::bin::BinaryMessage bin_msg_2; // binary message 2 config
	vn::bin::BinaryMessage bin_msg_3; // binary message 3 config

	// config parameters
	static constexpr int _params_vn100_port = 1; // default port for binary output messages
	static constexpr int _params_vn100_mode = 0; // default mode of the VN100 (0: external INS, 1: full INS)
	static constexpr int _params_vn100_msg1_rate = 2; // default rate divisor for binary output message 1
	static constexpr int _params_vn100_msg2_rate = 8; // default rate divisor for binary output message 2

};

// FIX: make this cleaner
extern VN100* vn100_instance; // global instance of the VN100 driver