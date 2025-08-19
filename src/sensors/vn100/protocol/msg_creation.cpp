/**
 * @file msg_creation.cpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Implements the message creation functions for the VectorNav VN100. This includes functions to create messages, calculate checksums,
 *	      and handle message formatting.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */


#include "msg_creation.hpp"

#include <stdio.h>

namespace vn
{

namespace msg
{

// message creation and formatting functions

/**
 * @brief Starts a new message with the given command header
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param cmd_header Command header to start the message with (e.g., "VNRRG")
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t start_message(buf_ref_t msg_buf, len_t msg_buf_size, const char *cmd_header)
{
	// ensure enough space is left in message buffer
	if (msg_buf_size < msg::HEADER_SIZE) {
		return -1;
	}

	// append '$XXXXX'
	return snprintf(msg_buf, msg::HEADER_SIZE, "$%s", cmd_header);
}

/**
 * @brief Adds a register ID field to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param reg_id Register ID to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_register_id(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, uint8_t reg_id)
{
	// comma + up to 3 digits
	constexpr len_t reg_field_size = 1 + 1 + 3;

	// ensure enough space is available
	if (msg_buf_size - msg_len < reg_field_size) {
		return -1;
	}

	// append ',XXX'
	return snprintf(msg_buf + msg_len, reg_field_size, ",%u", reg_id);
}

/**
 * @brief Adds a 8-bit unsigned integer field to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value 8-bit unsigned integer value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_uint8(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, uint8_t value)
{
	// comma + up to 3 digits (255)
	constexpr len_t uint8_field_size = 1 + 1 + 3;

	// ensure enough space is available
	if (msg_buf_size - msg_len < uint8_field_size) {
		return -1;
	}

	// append ',XXX'
	return snprintf(msg_buf + msg_len, uint8_field_size, ",%u", value);
}

/**
 * @brief Adds a 8-bit unsigned integer field in hex to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value 8-bit unsigned integer value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_uint8_hex(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, uint8_t value)
{
	// comma + 2 hex digits (0xFF)
	constexpr len_t uint8_hex_field_size = 1 + 1 + 2;

	// ensure enough space is available
	if (msg_buf_size - msg_len < uint8_hex_field_size) {
		return -1;
	}

	// append ',XX'
	return snprintf(msg_buf + msg_len, uint8_hex_field_size, ",%02X", value);
}

/**
 * @brief Adds a 16-bit unsigned integer field to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value 16-bit unsigned integer value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_uint16(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, uint16_t value)
{
	// comma + up to 5 digits (65535)
	constexpr len_t uint16_field_size = 1 + 1 + 5;

	// ensure enough space is available
	if (msg_buf_size - msg_len < uint16_field_size) {
		return -1;
	}

	// append ',XXXXX'
	return snprintf(msg_buf + msg_len, uint16_field_size, ",%u", value);
}

/**
 * @brief Adds a 16-bit unsigned integer field in hex to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value 16-bit unsigned integer value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_uint16_hex(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, uint16_t value)
{
	// comma + 4 hex digits (0xFFFF)
	constexpr len_t uint16_hex_field_size = 1 + 1 + 4;

	// ensure enough space is available
	if (msg_buf_size - msg_len < uint16_hex_field_size) {
		return -1;
	}

	// append ',XXXX'
	return snprintf(msg_buf + msg_len, uint16_hex_field_size, ",%04X", value);
}

/**
 * @brief Adds a 32-bit unsigned integer field to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value 32-bit unsigned integer value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_uint32(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, uint32_t value)
{
	// comma + up to 10 digits (4294967295)
	constexpr len_t uint32_field_size = 1 + 1 + 10;

	// ensure enough space is available
	if (msg_buf_size - msg_len < uint32_field_size) {
		return -1;
	}

	// append ',XXXXXXXXXX'
	return snprintf(msg_buf + msg_len, uint32_field_size, ",%lu", value);
}

/**
 * @brief Adds a float field to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value 32-bit floating point value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_float(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, float value)
{
	// NOTE: we place a cap of 10 digits on float values.
	// comma + up to 10 digits
	constexpr len_t float_field_size = 1 + 1 + 10;

	// ensure enough space is available
	if (msg_buf_size - msg_len < float_field_size) {
		return -1;
	}

	// append ',XXX.XXXXXX' (decimal floats)
	return snprintf(msg_buf + msg_len, float_field_size, ",%f", (double)value);
}

/**
 * @brief Adds a double field to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value 64-bit floating point value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_double(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, double value)
{
	// NOTE: we place a cap of 15 digits on double values.
	// comma + up to 15 digits
	constexpr len_t double_field_size = 1 + 1 + 15;

	// ensure enough space is available
	if (msg_buf_size - msg_len < double_field_size) {
		return -1;
	}

	// append ',XXXXX.XXXXXXXXX' (decimal floats)
	return snprintf(msg_buf, double_field_size, ",%lf", value);
}

/**
 * @brief Adds a string field to the message
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param value String value to add to the message
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t add_string(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, const char *value)
{
	// NOTE: max string size for any register argument is 24, some are less than that
	// comma + up to 24 chars
	constexpr len_t string_field_size = 1 + 1 + 24;

	// ensure enough space is available
	if (msg_buf_size - msg_len < string_field_size) {
		return -1;
	}

	// append ',XXXXXXXXXXXXXXXXXXXXXXXX'
	return snprintf(msg_buf + msg_len, string_field_size, ",%s", value);
}

/**
 * @brief Ends the current message and adds a checksum
 *
 * @param msg_buf Message buffer to write to
 * @param msg_buf_size Size of the message buffer
 * @param msg_len Length of the message so far
 * @param ignore_checksum Whether to ignore the checksum calculation, defaults to true
 * @return len_t Amount of bytes written to the buffer, or -1 on error
 */
static inline len_t end_message(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, bool ignore_checksum = false);
static inline len_t end_message(buf_ref_t msg_buf, len_t msg_buf_size, len_t msg_len, bool ignore_checksum)
{
	// '*' + 2 char checksum + '\n'
	constexpr len_t checksum_field_size = 1 + 1 + 2 + 1 + 1;

	// ensure enough space is available
	if (msg_buf_size - msg_len < checksum_field_size) {
		return -1;
	}

	// calculate checksum
	char checksum[2];

	if (ignore_checksum) {
		checksum[0] = CHECKSUM_IGNORE[0];
		checksum[1] = CHECKSUM_IGNORE[1];

	} else {
		calculate_checksum(checksum, (uint8_t *)msg_buf, msg_len);
	}

	return snprintf(msg_buf + msg_len, checksum_field_size, "*%s\n", checksum);
}

ErrorCode create_read_register(const uint8_t reg_id, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNRRG);

	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, reg_id);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_write_settings(buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWNV);
	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_restore_factory_settings(buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNRFS);
	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_reset(buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNRST);
	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_firmware_update(buf_ref_t msg_buf, len_t &msg_len)
{
	return ErrorCode::InvalidCommand; // not supported
}

ErrorCode create_known_magnetic_disturbance(const KnownMagneticDisturbance &mag_disturbance, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNKMD);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(mag_disturbance.state));

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_known_acceleration_disturbance(const KnownAccelerationDisturbance &accel_disturbance,
		buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNKAD);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(accel_disturbance.state));

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_async_output_enable(const AsyncOutputEnable &async_output_enable, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNASY);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, async_output_enable.enable);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_set_gyro_bias(buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNSGB);
	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_poll_binary_output_message(buf_ref_t msg_buf, len_t &msg_len)
{
	return ErrorCode::InvalidCommand; // not supported
}

// Configuration Registers
// Create functions make a fully assembled VNWRG message with the given register data

ErrorCode create_user_tag(const UserTag &user_tag, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, UserTag::ID);

	msg_len += add_string(msg_buf, MAX_MESSAGE_SIZE, msg_len, user_tag.tag);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_baudrate(const Baudrate &baudrate, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, Baudrate::ID);

	msg_len += add_uint32(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint32_t>(baudrate.baudrate));
	// TODO: support SerialPort option

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_async_data_output_type(const AsyncDataOutputType &async_data_output_type, buf_ref_t msg_buf,
					len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, AsyncDataOutputType::ID);

	msg_len += add_uint32(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint32_t>(async_data_output_type.ador));
	// TODO: support SerialPort option

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_async_data_output_freq(const AsyncDataOutputFreq &async_data_output_freq, buf_ref_t msg_buf,
					len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, AsyncDataOutputFreq::ID);

	msg_len += add_uint32(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint32_t>(async_data_output_freq.adof));
	// TODO: support SerialPort option

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_communication_protocol_control(const CommunicationProtocolControl &comm_protocol_control,
		buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, CommunicationProtocolControl::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			     static_cast<uint8_t>(comm_protocol_control.ascii_append_count));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			     static_cast<uint8_t>(comm_protocol_control.ascii_append_status));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(comm_protocol_control.spi_append_count));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(comm_protocol_control.spi_append_status));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(comm_protocol_control.ascii_checksum));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(comm_protocol_control.spi_checksum));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(comm_protocol_control.error_mode));

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_sync_control(const SyncControl &sync_control, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, SyncControl::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(sync_control.sync_in_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(sync_control.sync_in_edge));
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, sync_control.sync_in_skip_factor);
	msg_len += add_uint32(msg_buf, MAX_MESSAGE_SIZE, msg_len, sync_control.reserved1);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(sync_control.sync_out_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(sync_control.sync_out_polarity));
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, sync_control.sync_out_skip_factor);
	msg_len += add_uint32(msg_buf, MAX_MESSAGE_SIZE, msg_len, sync_control.sync_out_pulse_width);
	msg_len += add_uint32(msg_buf, MAX_MESSAGE_SIZE, msg_len, sync_control.reserved2);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_legacy_compatibility_settings(const LegacyCompatibilitySettings &legacy_settings, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, LegacyCompatibilitySettings::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, legacy_settings.reserved1);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, legacy_settings.reserved2);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, legacy_settings.imu_legacy);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, legacy_settings.hw_legacy);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_binary_output_message_config_1(const BinaryOutputMessageConfig1 &binary_output_msg_config_1,
		buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, BinaryOutputMessageConfig1::ID);
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			      static_cast<uint16_t>(binary_output_msg_config_1.async_mode));
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			      static_cast<uint16_t>(binary_output_msg_config_1.rate_divisor));
	msg_len += add_uint8_hex(msg_buf, MAX_MESSAGE_SIZE, msg_len, binary_output_msg_config_1.config.binary_group);

	for (int i = 0; i < 4; i++) {
		if (binary_output_msg_config_1.config.group_types[i] > 0) {
			msg_len += add_uint16_hex(msg_buf, MAX_MESSAGE_SIZE, msg_len,
						  binary_output_msg_config_1.config.group_types[i]);
		}
	}

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_binary_output_message_config_2(const BinaryOutputMessageConfig2 &binary_output_msg_config_2,
		buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, BinaryOutputMessageConfig2::ID);

	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			      static_cast<uint16_t>(binary_output_msg_config_2.async_mode));
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			      static_cast<uint16_t>(binary_output_msg_config_2.rate_divisor));
	msg_len += add_uint8_hex(msg_buf, MAX_MESSAGE_SIZE, msg_len, binary_output_msg_config_2.config.binary_group);

	for (int i = 0; i < 4; i++) {
		if (binary_output_msg_config_2.config.group_types[i] > 0) {
			msg_len += add_uint16_hex(msg_buf, MAX_MESSAGE_SIZE, msg_len,
						  binary_output_msg_config_2.config.group_types[i]);
		}
	}

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_binary_output_message_config_3(const BinaryOutputMessageConfig3 &binary_output_msg_config_3,
		buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, BinaryOutputMessageConfig3::ID);

	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			      static_cast<uint16_t>(binary_output_msg_config_3.async_mode));
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			      static_cast<uint16_t>(binary_output_msg_config_3.rate_divisor));
	msg_len += add_uint8_hex(msg_buf, MAX_MESSAGE_SIZE, msg_len, binary_output_msg_config_3.config.binary_group);

	for (int i = 0; i < 4; i++) {
		if (binary_output_msg_config_3.config.group_types[i] > 0) {
			msg_len += add_uint16_hex(msg_buf, MAX_MESSAGE_SIZE, msg_len,
						  binary_output_msg_config_3.config.group_types[i]);
		}
	}

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_magnetic_gravity_reference(const MagneticGravityReference &mag_grav_ref, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, MagneticGravityReference::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_grav_ref.mag_ref[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_grav_ref.mag_ref[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_grav_ref.mag_ref[2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_grav_ref.grav_ref[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_grav_ref.grav_ref[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_grav_ref.grav_ref[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_vpe_basic_control(const VpeBasicControl &vpe_basic_control, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, VpeBasicControl::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_basic_control.reserved);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(vpe_basic_control.heading_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(vpe_basic_control.filtering_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(vpe_basic_control.tuning_mode));

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_vpe_magnetometer_basic_tuning(const VpeMagnetometerBasicTuning &vpe_mag_tuning, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, VpeMagnetometerBasicTuning::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.base_tuning[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.base_tuning[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.base_tuning[2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.adaptive_tuning[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.adaptive_tuning[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.adaptive_tuning[2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.adaptive_filtering[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.adaptive_filtering[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_mag_tuning.adaptive_filtering[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_vpe_accelerometer_basic_tuning(const VpeAccelerometerBasicTuning &vpe_accel_tuning, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, VpeAccelerometerBasicTuning::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.base_tuning[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.base_tuning[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.base_tuning[2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.adaptive_tuning[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.adaptive_tuning[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.adaptive_tuning[2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.adaptive_filtering[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.adaptive_filtering[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, vpe_accel_tuning.adaptive_filtering[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_filter_startup_gyro_bias(const FilterStartupGyroBias &filter_startup_gyro_bias, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, FilterStartupGyroBias::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, filter_startup_gyro_bias.gyro_bias[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, filter_startup_gyro_bias.gyro_bias[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, filter_startup_gyro_bias.gyro_bias[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_magnetometer_calibration(const MagnetometerCalibration &mag_calibration, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, MagnetometerCalibration::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[0][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[0][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[0][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[1][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[1][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[1][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[2][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[2][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_gain[2][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_bias[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_bias[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, mag_calibration.mag_bias[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_accelerometer_calibration(const AccelerometerCalibration &accel_calibration, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, AccelerometerCalibration::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[0][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[0][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[0][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[1][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[1][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[1][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[2][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[2][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_gain[2][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_bias[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_bias[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, accel_calibration.acc_bias[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_reference_frame_rotation(const ReferenceFrameRotation &ref_frame_rotation, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, ReferenceFrameRotation::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[0][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[0][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[0][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[1][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[1][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[1][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[2][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[2][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_frame_rotation.rotation_matrix[2][2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_delta_theta_delta_velocity_config(const DeltaThetaDeltaVelocityConfig &delta_theta_config,
		buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, DeltaThetaDeltaVelocityConfig::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(delta_theta_config.integration_frame));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, delta_theta_config.gyro_compensation);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, delta_theta_config.acc_compensation);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, delta_theta_config.reserved1);
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, delta_theta_config.reserved2);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_gyro_calibration(const GyroCalibration &gyro_calibration, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, GyroCalibration::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[0][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[0][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[0][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[1][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[1][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[1][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[2][0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[2][1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_gain[2][2]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_bias[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_bias[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, gyro_calibration.gyro_bias[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_imu_filtering_config(const ImuFilteringConfig &imu_filtering_config, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, ImuFilteringConfig::ID);

	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, imu_filtering_config.mag_window_size);
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, imu_filtering_config.acc_window_size);
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, imu_filtering_config.gyro_window_size);
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, imu_filtering_config.temp_window_size);
	msg_len += add_uint16(msg_buf, MAX_MESSAGE_SIZE, msg_len, imu_filtering_config.pres_window_size);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(imu_filtering_config.mag_filter_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(imu_filtering_config.acc_filter_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(imu_filtering_config.gyro_filter_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(imu_filtering_config.temp_filter_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(imu_filtering_config.pres_filter_mode));

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_real_time_hsi_control(const RealTimeHsiControl &real_time_hsi_control, buf_ref_t msg_buf,
				       len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, RealTimeHsiControl::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, static_cast<uint8_t>(real_time_hsi_control.hsi_mode));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len,
			     static_cast<uint8_t>(real_time_hsi_control.apply_compensation));
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, real_time_hsi_control.converge_rate);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_velocity_aiding_measurement(const VelocityAidingMeasurement &velocity_aiding_measurements,
		buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, VelocityAidingMeasurement::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, velocity_aiding_measurements.velocity[0]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, velocity_aiding_measurements.velocity[1]);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, velocity_aiding_measurements.velocity[2]);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_velocity_aiding_control(const VelocityAidingControl &velocity_aiding_control, buf_ref_t msg_buf,
		len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, VelocityAidingControl::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, velocity_aiding_control.enable);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, velocity_aiding_control.vel_uncert_tuning);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, velocity_aiding_control.reserved);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_reference_model_config(const ReferenceModelConfig &ref_model_config, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, ReferenceModelConfig::ID);

	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.enable_mag_model);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.enable_gravity_model);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.reserved1);
	msg_len += add_uint8(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.reserved2);
	msg_len += add_uint32(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.recalc_threshold);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.year);
	msg_len += add_double(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.latitude);
	msg_len += add_double(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.longitude);
	msg_len += add_double(msg_buf, MAX_MESSAGE_SIZE, msg_len, ref_model_config.altitude);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

ErrorCode create_heave_basic_config(const HeaveBasicConfig &heave_basic_config, buf_ref_t msg_buf, len_t &msg_len)
{
	msg_len = start_message(msg_buf, MAX_MESSAGE_SIZE, VNWRG);
	msg_len += add_register_id(msg_buf, MAX_MESSAGE_SIZE, msg_len, HeaveBasicConfig::ID);

	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, heave_basic_config.initial_wave_period);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, heave_basic_config.initial_wave_amplitude);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, heave_basic_config.max_wave_period);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, heave_basic_config.min_wave_amplitude);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, heave_basic_config.delayed_heave_cutoff_freq);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, heave_basic_config.heave_cutoff_freq);
	msg_len += add_float(msg_buf, MAX_MESSAGE_SIZE, msg_len, heave_basic_config.heave_rate_cutoff_freq);

	msg_len += end_message(msg_buf, MAX_MESSAGE_SIZE, msg_len);

	return ErrorCode::OK;
}

// Measurement Registers (readonly)
// They dont get a create function, as they are read only.

} // namespace msg

} // namespace vn
