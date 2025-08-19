/**
 * @file vn100.cpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Implements the user side of the VectorNav VN100 driver, specifically the module interface. This includes module instantiation,
 *	      task spawning, option parsing, and the main run loop.
 * @date 2025-07
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#include "vn100.hpp"

VN100::VN100()
{
	// FIX: make set_port take a HardwareSerial pointer instead of a string
	// _uart.set_port();
}

VN100::~VN100()
{
	_uart.close();

	_uart.set_binary_callback(nullptr);
}

bool VN100::init()
{
	// initialize the uart line
	if (!_uart.open()) {
		Serial.println("Failed to open UART port");
		return false;
	}

	if (!ramp_up_baudrate()) {
		Serial.println("Failed to ramp up baudrate");
		return false;
	}

	// disable async output to start with. Having it enabled will flood the bus
	// we need the bus clear for our synchronous reads/writes at first
	vn::AsyncOutputEnable async_output_enable;
	async_output_enable.enable = false;

	if (_uart.write_register(async_output_enable) != vn::ErrorCode::OK) {
		Serial.println("Failed to write async output enable to VN100");
		return false;
	}

	vn::Model model;

	if (_uart.read_register(model) != vn::ErrorCode::OK) {
		Serial.println("Failed to read model information from VN100");
		return false;
	}

	vn::HardwareVersion hardware_version;

	if (_uart.read_register(hardware_version) != vn::ErrorCode::OK) {
		Serial.println("Failed to read hardware version from VN100");
		return false;
	}

	vn::SerialNumber serial_number;

	if (_uart.read_register(serial_number) != vn::ErrorCode::OK) {
		Serial.println("Failed to read serial number from VN100");
		return false;
	}

	vn::FirmwareVersion firmware_version;

	if (_uart.read_register(firmware_version) != vn::ErrorCode::OK) {
		Serial.println("Failed to read firmware version from VN100");
		return false;
	}

	// vn::UserTag set_user_tag;
	// strcpy(set_user_tag.tag, "Testing VN100");

	// if (_uart.write_register(set_user_tag) != vn::ErrorCode::OK) {
	// 	Serial.println("Failed to set user tag on VN100");
	// 	return false;
	// }

	vn::UserTag user_tag;

	if (_uart.read_register(user_tag) != vn::ErrorCode::OK) {
		Serial.println("Failed to read user tag from VN100");
		return false;
	}

	// manual Serial.println since it truncates the output
	Serial.printf("INFO  [VN100] Connected to VN100:\n"
	       "\tModel: %s\n"
	       "\tHardware Version: %lu\n"
	       "\tSerial Number: %lu\n"
	       "\tFirmware Version: %s\n"
	       "\tUser Tag: %s\n",
	       model.model, hardware_version.hardware_version,
	       serial_number.serial_number, firmware_version.firmware_version,
	       user_tag.tag);

	Serial.printf("VN100 initialized\n");

	// FIX: here is where you declare what messages the vn100 should send

	vn::BinaryOutputMessageConfig1 binary_output_msg_1;
	binary_output_msg_1.async_mode = static_cast<vn::AsyncMode>(_params_vn100_port);
	binary_output_msg_1.rate_divisor = static_cast<uint16_t>(_params_vn100_msg1_rate);

	bin_msg_1.binary_group = vn::bin::BinaryGroup::TimeGroup |
				 vn::bin::BinaryGroup::ImuGroup;
	bin_msg_1.group_types[0] = static_cast<uint16_t>(vn::bin::TimeTypes::TimeStartup);
	bin_msg_1.group_types[1] = static_cast<uint16_t>(vn::bin::ImuTypes::Accel) |
				   static_cast<uint16_t>(vn::bin::ImuTypes::AngularRate) |
				   static_cast<uint16_t>(vn::bin::ImuTypes::SensSat);

	binary_output_msg_1.config = bin_msg_1;

	if (_uart.write_register(binary_output_msg_1) != vn::ErrorCode::OK) {
		Serial.println("Failed to write binary output message config 1 to VN100");
		return false;
	}

	vn::BinaryOutputMessageConfig2 binary_output_msg_2;
	binary_output_msg_2.async_mode = static_cast<vn::AsyncMode>(_params_vn100_port);
	binary_output_msg_2.rate_divisor = static_cast<uint16_t>(_params_vn100_msg2_rate);

	bin_msg_2.binary_group = vn::bin::BinaryGroup::TimeGroup |
				 vn::bin::BinaryGroup::ImuGroup |
				 vn::bin::BinaryGroup::AttitudeGroup;
	bin_msg_2.group_types[0] = static_cast<uint16_t>(vn::bin::TimeTypes::TimeStartup);
	bin_msg_2.group_types[1] = static_cast<uint16_t>(vn::bin::ImuTypes::Temperature) |
				   static_cast<uint16_t>(vn::bin::ImuTypes::Pressure) |
				   static_cast<uint16_t>(vn::bin::ImuTypes::Mag);
	bin_msg_2.group_types[2] = static_cast<uint16_t>(vn::bin::AttitudeTypes::Quaternion) |
				   static_cast<uint16_t>(vn::bin::AttitudeTypes::LinAccelNed);

	binary_output_msg_2.config = bin_msg_2;

	if (_uart.write_register(binary_output_msg_2) != vn::ErrorCode::OK) {
		Serial.println("Failed to write binary output message config 2 to VN100");
		return false;
	}

	// TODO: message 3. this is not used but required (why) for hitl according to justin p

	_uart.set_binary_callback(binary_message_callback);

	// re-enable async output. synchronous reads/writes are done
	async_output_enable.enable = true;

	if (_uart.write_register(async_output_enable) != vn::ErrorCode::OK) {
		Serial.println("Failed to enable async output on VN100");
		return false;
	}

	_initialized = true; // mark as initialized
	return true;
}

bool VN100::ramp_up_baudrate()
{
	// end goal: set the baudrate to 921600
	// first: see whether we're already at 921600
	// if not, drop down to 115200 and then ramp up to 921600

	vn::Model model;
	vn::Baudrate configured_baudrate;

	constexpr vn::BaudrateSetting default_baudrate = vn::BaudrateSetting::BAUD_115200;
	constexpr vn::BaudrateSetting target_baudrate = vn::BaudrateSetting::BAUD_921600;

	// see whether we're already at the target baudrate
	_uart.set_baud(target_baudrate);

	if (_uart.read_register(model) == vn::ErrorCode::OK) {
		Serial.printf("VN100 already configured to baudrate: %lu\n", static_cast<uint32_t>(target_baudrate));
		return true; // we're already at the target baudrate

	} else {
		Serial.printf("VN100 not configured to target baudrate, trying to ramp up to: %lu\n",
			 static_cast<uint32_t>(target_baudrate));
	}

	// if not, try the default baudrate. If this fails, all is lost
	_uart.set_baud(default_baudrate);

	if (_uart.read_register(model) != vn::ErrorCode::OK) {
		Serial.printf("Failed to read model at default baudrate: %lu, cannot proceed with ramp up\n",
			static_cast<uint32_t>(default_baudrate));
		return false;
	}

	// now we can set the target baudrate
	configured_baudrate.baudrate = target_baudrate;

	if (_uart.write_register(configured_baudrate) != vn::ErrorCode::OK) {
		Serial.println("Failed to set baudrate to 921600 on VN100");
		return false;
	}

	_uart.set_baud(target_baudrate);

	// check if the baudrate was set correctly
	if (_uart.read_register(model) == vn::ErrorCode::OK) {
		Serial.printf("VN100 baudrate successfully set to: %lu\n", static_cast<uint32_t>(target_baudrate));
		return true;

	} else {
		Serial.printf("VN100 baudrate not set to 921600, currently at: %lu\n",
			static_cast<uint32_t>(configured_baudrate.baudrate));
	}

	return false;
}
