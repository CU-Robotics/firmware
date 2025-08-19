/**
 * @file msg_parsing.cpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Implements the message parsing functions for the VectorNav VN100. This includes functions to parse messages, validate checksums,
 *	      and handle message formatting. This also manages the conversion from raw message bytes to structs.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#include "msg_parsing.hpp"

#include <stdio.h>
#include <string.h>

namespace vn
{

namespace msg
{

ErrorCode extract_header(const_buf_ref_t msg_buf, len_t msg_len, MessageType *command, uint8_t *reg_id)
{
	// ensure the message length is valid
	if (msg_len > (signed)sizeof(msg_buf)) {
		return ErrorCode::InvalidParameter;
	}

	if (msg_len < HEADER_SIZE) {
		return ErrorCode::InvalidParameter;
	}

	// verify the start of the message
	if (msg_buf[0] != START) {
		return ErrorCode::InvalidParameter;
	}

	// extract the command type from the message
	char command_str[6] = {0}; // 5 characters + null terminator
	strncpy(command_str, msg_buf + 1, 5);

	// if input command is not null, assign the command type
	if (command != nullptr) {
		if (strcmp(command_str, VNRRG) == 0) {
			*command = MessageType::ReadRegister;

		} else if (strcmp(command_str, VNWRG) == 0) {
			*command = MessageType::WriteRegister;

		} else if (strcmp(command_str, VNWNV) == 0) {
			*command = MessageType::WriteSettings;

		} else if (strcmp(command_str, VNRFS) == 0) {
			*command = MessageType::RestoreFactorySettings;

		} else if (strcmp(command_str, VNRST) == 0) {
			*command = MessageType::Reset;

		} else if (strcmp(command_str, VNFWU) == 0) {
			*command = MessageType::FirmwareUpdate;

		} else if (strcmp(command_str, VNKMD) == 0) {
			*command = MessageType::KnownMagneticDisturbance;

		} else if (strcmp(command_str, VNKAD) == 0) {
			*command = MessageType::KnownAccelerationDisturbance;

		} else if (strcmp(command_str, VNASY) == 0) {
			*command = MessageType::AsyncOutputEnable;

		} else if (strcmp(command_str, VNSGB) == 0) {
			*command = MessageType::SetGyroBias;

		} else if (strcmp(command_str, VNBOM) == 0) {
			*command = MessageType::PollBinaryOutputMessage;

		} else if (strcmp(command_str, VNERR) == 0) {
			*command = MessageType::Error;

		} else {
			*command = MessageType::Unknown;
		}
	}

	// if the next character is not a delimiter, there is no register ID
	if (msg_buf[6] != DELIM) {
		if (reg_id != nullptr) {
			*reg_id = 255; // no register ID
		}

		return ErrorCode::OK;
	}

	// extract the register ID from the message
	if (reg_id != nullptr) {
		if (sscanf(msg_buf + HEADER_SIZE, "%hhu", reg_id) != 1) {
			return ErrorCode::InvalidParameter; // failed to parse register ID
		}
	}

	return ErrorCode::OK;
}

/**
 * @brief Extracts the checksum from a message buffer
 *
 * @param msg_buf The message buffer to extract the checksum from
 * @param msg_len The length of the message buffer
 * @param checksum The extracted checksum
 * @return ErrorCode indicating the result of the operation
 */
static inline ErrorCode extract_checksum(const_buf_ref_t msg_buf, len_t msg_len, char checksum[2])
{
	// ensure the message length is valid
	if (msg_len > (signed)sizeof(msg_buf)) {
		return ErrorCode::InvalidParameter;
	}

	// ensure the message is long enough to contain a checksum (at least 4 characters: '*' + 2 hex digits + '\n')
	if (msg_len < 4) {
		return ErrorCode::InvalidParameter;
	}

	// find the '*' character which indicates the start of the checksum
	const char *checksum_start = strchr(msg_buf, END_DATA);

	// verify start ptr is valid and contains enough characters
	if (checksum_start == nullptr || strlen(checksum_start) < 4) {
		return ErrorCode::InvalidParameter;
	}

	checksum[0] = checksum_start[1]; // first hex digit
	checksum[1] = checksum_start[2]; // second hex digit

	return ErrorCode::OK;
}

/**
 * @brief Extracts a string from a message buffer. This is done manually to accurately include whitespace.
 *
 * @param msg_buf The message buffer to extract the string from
 * @param msg_len The length of the message buffer
 * @param out_str The output buffer to store the extracted string
 * @param out_str_size The size of the output buffer
 * @return ErrorCode indicating the result of the operation
 */
static inline ErrorCode extract_string(const_buf_ref_t msg_buf, len_t msg_len, char *out_str, len_t out_str_size)
{
	// ensure the message length is valid
	if (msg_len > (signed)sizeof(msg_buf)) {
		return ErrorCode::InvalidParameter;
	}

	// loop through the msg_buf until we find either a DELIM, END_DATA, null terminator, or we reach the end of the buffer
	len_t i = 0;

	while (i < msg_len) {
		if (msg_buf[i] == DELIM || msg_buf[i] == END_DATA || msg_buf[i] == '\0') {
			break;
		}

		i++;
	}

	// ensure we found a valid delimiter
	if (i == msg_len) {
		return ErrorCode::InvalidParameter;
	}

	// copy the extracted string to the output buffer
	len_t copy_len = i < out_str_size ? i : out_str_size - 1;
	strncpy(out_str, reinterpret_cast<const char *>(msg_buf), copy_len);
	out_str[copy_len] = '\0'; // null-terminate the string

	return ErrorCode::OK;
}

ErrorCode verify_message(const_buf_ref_t msg_buf, len_t msg_len, uint8_t reg_id, MessageType command)
{
	// ensure the message length is valid
	if (msg_len > (signed)sizeof(msg_buf)) {
		return ErrorCode::InvalidParameter;
	}

	// extract the header information
	MessageType msg_command;
	uint8_t msg_reg_id;
	ErrorCode err = extract_header(msg_buf, msg_len, &msg_command, &msg_reg_id);

	if (err != ErrorCode::OK) {
		return err;
	}

	// if its an error frame, return the error code
	if (msg_command == MessageType::Error) {
		// extract the error code from the message
		int error_code;

		if (sscanf(msg_buf + HEADER_SIZE, "%x", &error_code) != 1) {
			return ErrorCode::InvalidParameter;
		}

		return static_cast<ErrorCode>(error_code);
	}

	// verify the command type if specified
	if (command != MessageType::Unknown && msg_command != command) {
		return ErrorCode::InvalidCommand;
	}

	// verify the register ID if specified
	if (reg_id != 255 && msg_reg_id != reg_id) {
		return ErrorCode::InvalidRegister;
	}

	// extract and verify the checksum
	char checksum[2];
	err = extract_checksum(msg_buf, msg_len, checksum);

	if (err != ErrorCode::OK) {
		return err;
	}

	if (!verify_checksum(checksum, reinterpret_cast<const uint8_t *>(msg_buf), msg_len)) {
		return ErrorCode::InvalidChecksum;
	}

	return ErrorCode::OK;
}

ErrorCode parse_write_settings(const_buf_ref_t msg_buf, len_t msg_len, WriteSettings &write_settings)
{
	ErrorCode err = verify_message(msg_buf, msg_len, 255, MessageType::WriteSettings);

	if (err != ErrorCode::OK) {
		return err;
	}

	// no data to parse for this command
	write_settings = WriteSettings();

	return ErrorCode::OK;
}

ErrorCode parse_restore_factory_settings(const_buf_ref_t msg_buf, len_t msg_len,
		RestoreFactorySettings &restore_factory_settings)
{
	ErrorCode err = verify_message(msg_buf, msg_len, 255, MessageType::RestoreFactorySettings);

	if (err != ErrorCode::OK) {
		return err;
	}

	// no data to parse for this command
	restore_factory_settings = RestoreFactorySettings();

	return ErrorCode::OK;
}

ErrorCode parse_reset(const_buf_ref_t msg_buf, len_t msg_len, Reset &reset)
{
	ErrorCode err = verify_message(msg_buf, msg_len, 255, MessageType::Reset);

	if (err != ErrorCode::OK) {
		return err;
	}

	// no data to parse for this command
	reset = Reset();

	return ErrorCode::OK;
}

ErrorCode parse_firmware_update(const_buf_ref_t msg_buf, len_t msg_len, FirmwareUpdate &firmware_update)
{
	// not supported
	return ErrorCode::InvalidCommand;
}

ErrorCode parse_known_magnetic_disturbance(const_buf_ref_t msg_buf, len_t msg_len,
		KnownMagneticDisturbance &mag_disturbance)
{
	ErrorCode err = verify_message(msg_buf, msg_len, 255, MessageType::KnownMagneticDisturbance);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after command header
	char *data_start = strchr(msg_buf, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, KnownMagneticDisturbance::FORMAT,
		   reinterpret_cast<uint8_t *>(&mag_disturbance.state)) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_known_acceleration_disturbance(const_buf_ref_t msg_buf, len_t msg_len,
		KnownAccelerationDisturbance &accel_disturbance)
{
	ErrorCode err = verify_message(msg_buf, msg_len, 255, MessageType::KnownAccelerationDisturbance);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after command header
	char *data_start = strchr(msg_buf, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, KnownAccelerationDisturbance::FORMAT,
		   reinterpret_cast<uint8_t *>(&accel_disturbance.state)) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_async_output_enable(const_buf_ref_t msg_buf, len_t msg_len, AsyncOutputEnable &async_output_enable)
{
	ErrorCode err = verify_message(msg_buf, msg_len, 255, MessageType::AsyncOutputEnable);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after command header
	char *data_start = strchr(msg_buf, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, AsyncOutputEnable::FORMAT, &async_output_enable.enable) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_set_gyro_bias(const_buf_ref_t msg_buf, len_t msg_len, SetGyroBias &set_gyro_bias)
{
	ErrorCode err = verify_message(msg_buf, msg_len, 255, MessageType::SetGyroBias);

	if (err != ErrorCode::OK) {
		return err;
	}

	// no data to parse for this command
	set_gyro_bias = SetGyroBias();

	return ErrorCode::OK;
}

ErrorCode parse_poll_binary_output_message(const_buf_ref_t msg_buf, len_t msg_len,
		PollBinaryOutputMessage &poll_binary_output_message)
{
	// not supported
	return ErrorCode::InvalidCommand;
}

ErrorCode parse_user_tag(const_buf_ref_t msg_buf, len_t msg_len, UserTag &user_tag)
{
	ErrorCode err = verify_message(msg_buf, msg_len, UserTag::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	len_t remaining_len = msg_len - (data_start - msg_buf + 1);
	err = extract_string(*reinterpret_cast<const buf_t *>(data_start + 1), remaining_len, user_tag.tag,
			     sizeof(user_tag.tag));

	if (err != ErrorCode::OK) {
		return err;
	}

	return ErrorCode::OK;
}

ErrorCode parse_baudrate(const_buf_ref_t msg_buf, len_t msg_len, Baudrate &baudrate)
{
	ErrorCode err = verify_message(msg_buf, msg_len, Baudrate::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, Baudrate::FORMAT,
		   reinterpret_cast<uint32_t *>(&baudrate.baudrate)) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_async_data_output_type(const_buf_ref_t msg_buf, len_t msg_len,
				       AsyncDataOutputType &async_data_output_type)
{
	ErrorCode err = verify_message(msg_buf, msg_len, AsyncDataOutputType::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, AsyncDataOutputType::FORMAT,
		   reinterpret_cast<uint32_t *>(&async_data_output_type.ador)) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_async_data_output_freq(const_buf_ref_t msg_buf, len_t msg_len,
				       AsyncDataOutputFreq &async_data_output_freq)
{
	ErrorCode err = verify_message(msg_buf, msg_len, AsyncDataOutputFreq::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, AsyncDataOutputFreq::FORMAT,
		   reinterpret_cast<uint32_t *>(&async_data_output_freq.adof)) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_communication_protocol_control(const_buf_ref_t msg_buf, len_t msg_len,
		CommunicationProtocolControl &comm_protocol_control)
{
	ErrorCode err = verify_message(msg_buf, msg_len, CommunicationProtocolControl::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, CommunicationProtocolControl::FORMAT,
		   reinterpret_cast<uint8_t *>(&comm_protocol_control.ascii_append_count),
		   reinterpret_cast<uint8_t *>(&comm_protocol_control.ascii_append_status),
		   reinterpret_cast<uint8_t *>(&comm_protocol_control.spi_append_count),
		   reinterpret_cast<uint8_t *>(&comm_protocol_control.spi_append_status),
		   reinterpret_cast<uint8_t *>(&comm_protocol_control.ascii_checksum),
		   reinterpret_cast<uint8_t *>(&comm_protocol_control.spi_checksum),
		   reinterpret_cast<uint8_t *>(&comm_protocol_control.error_mode)
		  ) != 7) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_sync_control(const_buf_ref_t msg_buf, len_t msg_len, SyncControl &sync_control)
{
	ErrorCode err = verify_message(msg_buf, msg_len, SyncControl::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, SyncControl::FORMAT,
		   reinterpret_cast<uint8_t *>(&sync_control.sync_in_mode),
		   reinterpret_cast<uint8_t *>(&sync_control.sync_in_edge),
		   &sync_control.sync_in_skip_factor,
		   &sync_control.reserved1,
		   reinterpret_cast<uint8_t *>(&sync_control.sync_out_mode),
		   reinterpret_cast<uint8_t *>(&sync_control.sync_out_polarity),
		   &sync_control.sync_out_skip_factor,
		   &sync_control.sync_out_pulse_width,
		   &sync_control.reserved2
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_legacy_compatibility_settings(const_buf_ref_t msg_buf, len_t msg_len,
		LegacyCompatibilitySettings &legacy_settings)
{
	ErrorCode err = verify_message(msg_buf, msg_len, LegacyCompatibilitySettings::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, LegacyCompatibilitySettings::FORMAT,
		   &legacy_settings.reserved1,
		   &legacy_settings.reserved2,
		   &legacy_settings.imu_legacy,
		   &legacy_settings.hw_legacy
		  ) != 4) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_binary_output_message_config_1(const_buf_ref_t msg_buf, len_t msg_len,
		BinaryOutputMessageConfig1 &binary_output_msg_config)
{
	ErrorCode err = verify_message(msg_buf, msg_len, BinaryOutputMessageConfig1::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	// try to parse all parts. we might not get all of the OutputTypes, but we should at least get the async mode, rate divisor, binary group, and one group type
	if (sscanf(data_start + 1, BinaryOutputMessageConfig1::FORMAT,
		   reinterpret_cast<uint16_t *>(&binary_output_msg_config.async_mode),
		   &binary_output_msg_config.rate_divisor,
		   reinterpret_cast<uint8_t *>(&binary_output_msg_config.config.binary_group),
		   &binary_output_msg_config.config.group_types[0],
		   &binary_output_msg_config.config.group_types[1],
		   &binary_output_msg_config.config.group_types[2],
		   &binary_output_msg_config.config.group_types[3]
		  ) < 4) {
		return ErrorCode::InvalidParameter; // failed to parse all required fields
	}

	return ErrorCode::OK;
}

ErrorCode parse_binary_output_message_config_2(const_buf_ref_t msg_buf, len_t msg_len,
		BinaryOutputMessageConfig2 &binary_output_msg_config)
{
	ErrorCode err = verify_message(msg_buf, msg_len, BinaryOutputMessageConfig2::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	// try to parse all parts. we might not get all of the OutputTypes, but we should at least get the async mode, rate divisor, binary group, and one group type
	if (sscanf(data_start + 1, BinaryOutputMessageConfig2::FORMAT,
		   reinterpret_cast<uint16_t *>(&binary_output_msg_config.async_mode),
		   &binary_output_msg_config.rate_divisor,
		   reinterpret_cast<uint8_t *>(&binary_output_msg_config.config.binary_group),
		   &binary_output_msg_config.config.group_types[0],
		   &binary_output_msg_config.config.group_types[1],
		   &binary_output_msg_config.config.group_types[2],
		   &binary_output_msg_config.config.group_types[3]
		  ) < 4) {
		return ErrorCode::InvalidParameter; // failed to parse all required fields
	}

	return ErrorCode::OK;
}

ErrorCode parse_binary_output_message_config_3(const_buf_ref_t msg_buf, len_t msg_len,
		BinaryOutputMessageConfig3 &binary_output_msg_config)
{
	ErrorCode err = verify_message(msg_buf, msg_len, BinaryOutputMessageConfig3::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	// try to parse all parts. we might not get all of the OutputTypes, but we should at least get the async mode, rate divisor, binary group, and one group type
	if (sscanf(data_start + 1, BinaryOutputMessageConfig3::FORMAT,
		   reinterpret_cast<uint16_t *>(&binary_output_msg_config.async_mode),
		   &binary_output_msg_config.rate_divisor,
		   reinterpret_cast<uint8_t *>(&binary_output_msg_config.config.binary_group),
		   &binary_output_msg_config.config.group_types[0],
		   &binary_output_msg_config.config.group_types[1],
		   &binary_output_msg_config.config.group_types[2],
		   &binary_output_msg_config.config.group_types[3]
		  ) < 4) {
		return ErrorCode::InvalidParameter; // failed to parse all required fields
	}

	return ErrorCode::OK;
}

ErrorCode parse_magnetic_gravity_reference(const_buf_ref_t msg_buf, len_t msg_len,
		MagneticGravityReference &mag_grav_ref)
{
	ErrorCode err = verify_message(msg_buf, msg_len, MagneticGravityReference::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, MagneticGravityReference::FORMAT,
		   &mag_grav_ref.mag_ref[0],
		   &mag_grav_ref.mag_ref[1],
		   &mag_grav_ref.mag_ref[2],
		   &mag_grav_ref.grav_ref[0],
		   &mag_grav_ref.grav_ref[1],
		   &mag_grav_ref.grav_ref[2]
		  ) != 6) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_vpe_basic_control(const_buf_ref_t msg_buf, len_t msg_len,
				  VpeBasicControl &vpe_basic_control)
{
	ErrorCode err = verify_message(msg_buf, msg_len, VpeBasicControl::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, VpeBasicControl::FORMAT,
		   &vpe_basic_control.reserved,
		   reinterpret_cast<uint8_t *>(&vpe_basic_control.heading_mode),
		   reinterpret_cast<uint8_t *>(&vpe_basic_control.filtering_mode),
		   reinterpret_cast<uint8_t *>(&vpe_basic_control.tuning_mode)
		  ) != 4) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_vpe_magnetometer_basic_tuning(const_buf_ref_t msg_buf, len_t msg_len,
		VpeMagnetometerBasicTuning &vpe_mag_tuning)
{
	ErrorCode err = verify_message(msg_buf, msg_len, VpeMagnetometerBasicTuning::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, VpeMagnetometerBasicTuning::FORMAT,
		   &vpe_mag_tuning.base_tuning[0],
		   &vpe_mag_tuning.base_tuning[1],
		   &vpe_mag_tuning.base_tuning[2],
		   &vpe_mag_tuning.adaptive_tuning[0],
		   &vpe_mag_tuning.adaptive_tuning[1],
		   &vpe_mag_tuning.adaptive_tuning[2],
		   &vpe_mag_tuning.adaptive_filtering[0],
		   &vpe_mag_tuning.adaptive_filtering[1],
		   &vpe_mag_tuning.adaptive_filtering[2]
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_vpe_accelerometer_basic_tuning(const_buf_ref_t msg_buf, len_t msg_len,
		VpeAccelerometerBasicTuning &vpe_accel_tuning)
{
	ErrorCode err = verify_message(msg_buf, msg_len, VpeAccelerometerBasicTuning::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, VpeAccelerometerBasicTuning::FORMAT,
		   &vpe_accel_tuning.base_tuning[0],
		   &vpe_accel_tuning.base_tuning[1],
		   &vpe_accel_tuning.base_tuning[2],
		   &vpe_accel_tuning.adaptive_tuning[0],
		   &vpe_accel_tuning.adaptive_tuning[1],
		   &vpe_accel_tuning.adaptive_tuning[2],
		   &vpe_accel_tuning.adaptive_filtering[0],
		   &vpe_accel_tuning.adaptive_filtering[1],
		   &vpe_accel_tuning.adaptive_filtering[2]
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_filter_startup_gyro_bias(const_buf_ref_t msg_buf, len_t msg_len,
		FilterStartupGyroBias &filter_startup_gyro_bias)
{
	ErrorCode err = verify_message(msg_buf, msg_len, FilterStartupGyroBias::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, FilterStartupGyroBias::FORMAT,
		   &filter_startup_gyro_bias.gyro_bias[0],
		   &filter_startup_gyro_bias.gyro_bias[1],
		   &filter_startup_gyro_bias.gyro_bias[2]
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_magnetometer_calibration(const_buf_ref_t msg_buf, len_t msg_len,
		MagnetometerCalibration &mag_calibration)
{
	ErrorCode err = verify_message(msg_buf, msg_len, MagnetometerCalibration::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, MagnetometerCalibration::FORMAT,
		   &mag_calibration.mag_gain[0][0],
		   &mag_calibration.mag_gain[0][1],
		   &mag_calibration.mag_gain[0][2],
		   &mag_calibration.mag_gain[1][0],
		   &mag_calibration.mag_gain[1][1],
		   &mag_calibration.mag_gain[1][2],
		   &mag_calibration.mag_gain[2][0],
		   &mag_calibration.mag_gain[2][1],
		   &mag_calibration.mag_gain[2][2],
		   &mag_calibration.mag_bias[0],
		   &mag_calibration.mag_bias[1],
		   &mag_calibration.mag_bias[2]
		  ) != 12) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_accelerometer_calibration(const_buf_ref_t msg_buf, len_t msg_len,
		AccelerometerCalibration &accel_calibration)
{
	ErrorCode err = verify_message(msg_buf, msg_len, AccelerometerCalibration::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, AccelerometerCalibration::FORMAT,
		   &accel_calibration.acc_gain[0][0],
		   &accel_calibration.acc_gain[0][1],
		   &accel_calibration.acc_gain[0][2],
		   &accel_calibration.acc_gain[1][0],
		   &accel_calibration.acc_gain[1][1],
		   &accel_calibration.acc_gain[1][2],
		   &accel_calibration.acc_gain[2][0],
		   &accel_calibration.acc_gain[2][1],
		   &accel_calibration.acc_gain[2][2],
		   &accel_calibration.acc_bias[0],
		   &accel_calibration.acc_bias[1],
		   &accel_calibration.acc_bias[2]
		  ) != 12) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_reference_frame_rotation(const_buf_ref_t msg_buf, len_t msg_len,
		ReferenceFrameRotation &ref_frame_rotation)
{
	ErrorCode err = verify_message(msg_buf, msg_len, ReferenceFrameRotation::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, ReferenceFrameRotation::FORMAT,
		   &ref_frame_rotation.rotation_matrix[0][0],
		   &ref_frame_rotation.rotation_matrix[0][1],
		   &ref_frame_rotation.rotation_matrix[0][2],
		   &ref_frame_rotation.rotation_matrix[1][0],
		   &ref_frame_rotation.rotation_matrix[1][1],
		   &ref_frame_rotation.rotation_matrix[1][2],
		   &ref_frame_rotation.rotation_matrix[2][0],
		   &ref_frame_rotation.rotation_matrix[2][1],
		   &ref_frame_rotation.rotation_matrix[2][2]
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_delta_theta_delta_velocity_config(const_buf_ref_t msg_buf, len_t msg_len,
		DeltaThetaDeltaVelocityConfig &delta_theta_config)
{
	ErrorCode err = verify_message(msg_buf, msg_len, DeltaThetaDeltaVelocityConfig::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, DeltaThetaDeltaVelocityConfig::FORMAT,
		   reinterpret_cast<uint8_t *>(&delta_theta_config.integration_frame),
		   &delta_theta_config.gyro_compensation,
		   &delta_theta_config.acc_compensation,
		   &delta_theta_config.reserved1,
		   &delta_theta_config.reserved2
		  ) != 5) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_gyro_calibration(const_buf_ref_t msg_buf, len_t msg_len,
				 GyroCalibration &gyro_calibration)
{
	ErrorCode err = verify_message(msg_buf, msg_len, GyroCalibration::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, GyroCalibration::FORMAT,
		   &gyro_calibration.gyro_gain[0][0],
		   &gyro_calibration.gyro_gain[0][1],
		   &gyro_calibration.gyro_gain[0][2],
		   &gyro_calibration.gyro_gain[1][0],
		   &gyro_calibration.gyro_gain[1][1],
		   &gyro_calibration.gyro_gain[1][2],
		   &gyro_calibration.gyro_gain[2][0],
		   &gyro_calibration.gyro_gain[2][1],
		   &gyro_calibration.gyro_gain[2][2],
		   &gyro_calibration.gyro_bias[0],
		   &gyro_calibration.gyro_bias[1],
		   &gyro_calibration.gyro_bias[2]
		  ) != 12) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_imu_filtering_config(const_buf_ref_t msg_buf, len_t msg_len,
				     ImuFilteringConfig &imu_filtering_config)
{
	ErrorCode err = verify_message(msg_buf, msg_len, ImuFilteringConfig::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, ImuFilteringConfig::FORMAT,
		   &imu_filtering_config.mag_window_size,
		   &imu_filtering_config.acc_window_size,
		   &imu_filtering_config.gyro_window_size,
		   &imu_filtering_config.temp_window_size,
		   &imu_filtering_config.pres_window_size,
		   reinterpret_cast<uint8_t *>(&imu_filtering_config.mag_filter_mode),
		   reinterpret_cast<uint8_t *>(&imu_filtering_config.acc_filter_mode),
		   reinterpret_cast<uint8_t *>(&imu_filtering_config.gyro_filter_mode),
		   reinterpret_cast<uint8_t *>(&imu_filtering_config.temp_filter_mode),
		   reinterpret_cast<uint8_t *>(&imu_filtering_config.pres_filter_mode)
		  ) != 10) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_real_time_hsi_control(const_buf_ref_t msg_buf, len_t msg_len,
				      RealTimeHsiControl &real_time_hsi_control)
{
	ErrorCode err = verify_message(msg_buf, msg_len, RealTimeHsiControl::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, RealTimeHsiControl::FORMAT,
		   reinterpret_cast<uint8_t *>(&real_time_hsi_control.hsi_mode),
		   reinterpret_cast<uint8_t *>(&real_time_hsi_control.apply_compensation),
		   &real_time_hsi_control.converge_rate
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_velocity_aiding_measurement(const_buf_ref_t msg_buf, len_t msg_len,
		VelocityAidingMeasurement &velocity_aiding_measurement)
{
	ErrorCode err = verify_message(msg_buf, msg_len, VelocityAidingMeasurement::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, VelocityAidingMeasurement::FORMAT,
		   &velocity_aiding_measurement.velocity[0],
		   &velocity_aiding_measurement.velocity[1],
		   &velocity_aiding_measurement.velocity[2]
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_velocity_aiding_control(const_buf_ref_t msg_buf, len_t msg_len,
					VelocityAidingControl &velocity_aiding_control)
{
	ErrorCode err = verify_message(msg_buf, msg_len, VelocityAidingControl::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, VelocityAidingControl::FORMAT,
		   &velocity_aiding_control.enable,
		   &velocity_aiding_control.vel_uncert_tuning,
		   &velocity_aiding_control.reserved
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_reference_model_config(const_buf_ref_t msg_buf, len_t msg_len,
				       ReferenceModelConfig &ref_model_config)
{
	ErrorCode err = verify_message(msg_buf, msg_len, ReferenceModelConfig::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, ReferenceModelConfig::FORMAT,
		   &ref_model_config.enable_mag_model,
		   &ref_model_config.enable_gravity_model,
		   &ref_model_config.reserved1,
		   &ref_model_config.reserved2,
		   &ref_model_config.recalc_threshold,
		   &ref_model_config.year,
		   &ref_model_config.latitude,
		   &ref_model_config.longitude,
		   &ref_model_config.altitude
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_heave_basic_config(const_buf_ref_t msg_buf, len_t msg_len,
				   HeaveBasicConfig &heave_basic_config)
{
	ErrorCode err = verify_message(msg_buf, msg_len, HeaveBasicConfig::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, HeaveBasicConfig::FORMAT,
		   &heave_basic_config.initial_wave_period,
		   &heave_basic_config.initial_wave_amplitude,
		   &heave_basic_config.max_wave_period,
		   &heave_basic_config.min_wave_amplitude,
		   &heave_basic_config.delayed_heave_cutoff_freq,
		   &heave_basic_config.heave_cutoff_freq,
		   &heave_basic_config.heave_rate_cutoff_freq
		  ) != 7) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_model(const_buf_ref_t msg_buf, len_t msg_len, Model &model)
{
	ErrorCode err = verify_message(msg_buf, msg_len, Model::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, Model::FORMAT, model.model) != 1) {
		return ErrorCode::InvalidParameter;
	}

	len_t remaining_len = msg_len - (data_start - msg_buf + 1);
	err = extract_string(*reinterpret_cast<const buf_t *>(data_start + 1), remaining_len, model.model, sizeof(model.model));

	if (err != ErrorCode::OK) {
		return err;
	}

	return ErrorCode::OK;
}

ErrorCode parse_hardware_version(const_buf_ref_t msg_buf, len_t msg_len,
				 HardwareVersion &hardware_version)
{
	ErrorCode err = verify_message(msg_buf, msg_len, HardwareVersion::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, HardwareVersion::FORMAT, &hardware_version.hardware_version) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_serial_number(const_buf_ref_t msg_buf, len_t msg_len, SerialNumber &serial_number)
{
	ErrorCode err = verify_message(msg_buf, msg_len, SerialNumber::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, SerialNumber::FORMAT, &serial_number.serial_number) != 1) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_firmware_version(const_buf_ref_t msg_buf, len_t msg_len,
				 FirmwareVersion &firmware_version)
{
	ErrorCode err = verify_message(msg_buf, msg_len, FirmwareVersion::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, FirmwareVersion::FORMAT, firmware_version.firmware_version) != 1) {
		return ErrorCode::InvalidParameter;
	}

	len_t remaining_len = msg_len - (data_start - msg_buf + 1);
	err = extract_string(*reinterpret_cast<const buf_t *>(data_start + 1), remaining_len, firmware_version.firmware_version,
			     sizeof(firmware_version.firmware_version));

	if (err != ErrorCode::OK) {
		return err;
	}

	return ErrorCode::OK;
}

ErrorCode parse_sync_status(const_buf_ref_t msg_buf, len_t msg_len, SyncStatus &sync_status)
{
	ErrorCode err = verify_message(msg_buf, msg_len, SyncStatus::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, SyncStatus::FORMAT,
		   &sync_status.sync_in_count,
		   &sync_status.sync_in_time,
		   &sync_status.sync_out_count
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_yaw_pitch_roll(const_buf_ref_t msg_buf, len_t msg_len, YawPitchRoll &yaw_pitch_roll)
{
	ErrorCode err = verify_message(msg_buf, msg_len, YawPitchRoll::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, YawPitchRoll::FORMAT,
		   &yaw_pitch_roll.yaw,
		   &yaw_pitch_roll.pitch,
		   &yaw_pitch_roll.roll
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_quaternion(const_buf_ref_t msg_buf, len_t msg_len, Quaternion &quaternion)
{
	ErrorCode err = verify_message(msg_buf, msg_len, Quaternion::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, Quaternion::FORMAT,
		   &quaternion.quaternion[0],
		   &quaternion.quaternion[1],
		   &quaternion.quaternion[2],
		   &quaternion.quaternion[3]
		  ) != 4) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_quaternion_compensated_imu(const_buf_ref_t msg_buf, len_t msg_len,
		QuaternionCompensatedImu &quaternion_compensated_imu)
{
	ErrorCode err = verify_message(msg_buf, msg_len, QuaternionCompensatedImu::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, QuaternionCompensatedImu::FORMAT,
		   &quaternion_compensated_imu.quaternion[0],
		   &quaternion_compensated_imu.quaternion[1],
		   &quaternion_compensated_imu.quaternion[2],
		   &quaternion_compensated_imu.quaternion[3],
		   &quaternion_compensated_imu.mag[0],
		   &quaternion_compensated_imu.mag[1],
		   &quaternion_compensated_imu.mag[2],
		   &quaternion_compensated_imu.acc[0],
		   &quaternion_compensated_imu.acc[1],
		   &quaternion_compensated_imu.acc[2],
		   &quaternion_compensated_imu.gyro[0],
		   &quaternion_compensated_imu.gyro[1],
		   &quaternion_compensated_imu.gyro[2]
		  ) != 13) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_yaw_pitch_roll_compensated_imu(const_buf_ref_t msg_buf, len_t msg_len,
		YawPitchRollCompensatedImu &yaw_pitch_roll_compensated_imu)
{
	ErrorCode err = verify_message(msg_buf, msg_len, YawPitchRollCompensatedImu::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, YawPitchRollCompensatedImu::FORMAT,
		   &yaw_pitch_roll_compensated_imu.yaw,
		   &yaw_pitch_roll_compensated_imu.pitch,
		   &yaw_pitch_roll_compensated_imu.roll,
		   &yaw_pitch_roll_compensated_imu.mag[0],
		   &yaw_pitch_roll_compensated_imu.mag[1],
		   &yaw_pitch_roll_compensated_imu.mag[2],
		   &yaw_pitch_roll_compensated_imu.acc[0],
		   &yaw_pitch_roll_compensated_imu.acc[1],
		   &yaw_pitch_roll_compensated_imu.acc[2],
		   &yaw_pitch_roll_compensated_imu.gyro[0],
		   &yaw_pitch_roll_compensated_imu.gyro[1],
		   &yaw_pitch_roll_compensated_imu.gyro[2]
		  ) != 12) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_yaw_pitch_roll_linear_accel_gyro(const_buf_ref_t msg_buf, len_t msg_len,
		YawPitchRollLinearAccelGyro &yaw_pitch_roll_linear_accel_gyro)
{
	ErrorCode err = verify_message(msg_buf, msg_len, YawPitchRollLinearAccelGyro::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, YawPitchRollLinearAccelGyro::FORMAT,
		   &yaw_pitch_roll_linear_accel_gyro.yaw,
		   &yaw_pitch_roll_linear_accel_gyro.pitch,
		   &yaw_pitch_roll_linear_accel_gyro.roll,
		   &yaw_pitch_roll_linear_accel_gyro.lin_accel[0],
		   &yaw_pitch_roll_linear_accel_gyro.lin_accel[1],
		   &yaw_pitch_roll_linear_accel_gyro.lin_accel[2],
		   &yaw_pitch_roll_linear_accel_gyro.gyro[0],
		   &yaw_pitch_roll_linear_accel_gyro.gyro[1],
		   &yaw_pitch_roll_linear_accel_gyro.gyro[2]
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_yaw_pitch_roll_linear_inertial_accel_gyro(const_buf_ref_t msg_buf, len_t msg_len,
		YawPitchRollLinearInertialAccelGyro &yaw_pitch_roll_linear_inertial_accel_gyro)
{
	ErrorCode err = verify_message(msg_buf, msg_len, YawPitchRollLinearInertialAccelGyro::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, YawPitchRollLinearInertialAccelGyro::FORMAT,
		   &yaw_pitch_roll_linear_inertial_accel_gyro.yaw,
		   &yaw_pitch_roll_linear_inertial_accel_gyro.pitch,
		   &yaw_pitch_roll_linear_inertial_accel_gyro.roll,
		   &yaw_pitch_roll_linear_inertial_accel_gyro.lin_accel[0],
		   &yaw_pitch_roll_linear_inertial_accel_gyro.lin_accel[1],
		   &yaw_pitch_roll_linear_inertial_accel_gyro.lin_accel[2],
		   &yaw_pitch_roll_linear_inertial_accel_gyro.gyro[0],
		   &yaw_pitch_roll_linear_inertial_accel_gyro.gyro[1],
		   &yaw_pitch_roll_linear_inertial_accel_gyro.gyro[2]
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_compensated_magnetometer(const_buf_ref_t msg_buf, len_t msg_len,
		CompensatedMagnetometer &compensated_magnetometer)
{
	ErrorCode err = verify_message(msg_buf, msg_len, CompensatedMagnetometer::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, CompensatedMagnetometer::FORMAT,
		   &compensated_magnetometer.mag[0],
		   &compensated_magnetometer.mag[1],
		   &compensated_magnetometer.mag[2]
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_compensated_accelerometer(const_buf_ref_t msg_buf, len_t msg_len,
		CompensatedAccelerometer &compensated_accelerometer)
{
	ErrorCode err = verify_message(msg_buf, msg_len, CompensatedAccelerometer::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, CompensatedAccelerometer::FORMAT,
		   &compensated_accelerometer.acc[0],
		   &compensated_accelerometer.acc[1],
		   &compensated_accelerometer.acc[2]
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_compensated_gyro(const_buf_ref_t msg_buf, len_t msg_len,
				 CompensatedGyro &compensated_gyro)
{
	ErrorCode err = verify_message(msg_buf, msg_len, CompensatedGyro::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, CompensatedGyro::FORMAT,
		   &compensated_gyro.gyro[0],
		   &compensated_gyro.gyro[1],
		   &compensated_gyro.gyro[2]
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_compensated_imu(const_buf_ref_t msg_buf, len_t msg_len, CompensatedImu &compensated_imu)
{
	ErrorCode err = verify_message(msg_buf, msg_len, CompensatedImu::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, CompensatedImu::FORMAT,
		   &compensated_imu.mag[0],
		   &compensated_imu.mag[1],
		   &compensated_imu.mag[2],
		   &compensated_imu.acc[0],
		   &compensated_imu.acc[1],
		   &compensated_imu.acc[2],
		   &compensated_imu.gyro[0],
		   &compensated_imu.gyro[1],
		   &compensated_imu.gyro[2]
		  ) != 9) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_imu_measurements(const_buf_ref_t msg_buf, len_t msg_len,
				 ImuMeasurements &imu_measurements)
{
	ErrorCode err = verify_message(msg_buf, msg_len, ImuMeasurements::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, ImuMeasurements::FORMAT,
		   &imu_measurements.uncomp_mag[0],
		   &imu_measurements.uncomp_mag[1],
		   &imu_measurements.uncomp_mag[2],
		   &imu_measurements.uncomp_acc[0],
		   &imu_measurements.uncomp_acc[1],
		   &imu_measurements.uncomp_acc[2],
		   &imu_measurements.uncomp_gyro[0],
		   &imu_measurements.uncomp_gyro[1],
		   &imu_measurements.uncomp_gyro[2],
		   &imu_measurements.temp,
		   &imu_measurements.pres
		  ) != 11) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_delta_theta_delta_velocity(const_buf_ref_t msg_buf, len_t msg_len,
		DeltaThetaDeltaVelocity &delta_theta_delta_velocity)
{
	ErrorCode err = verify_message(msg_buf, msg_len, DeltaThetaDeltaVelocity::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, DeltaThetaDeltaVelocity::FORMAT,
		   &delta_theta_delta_velocity.delta_time,
		   &delta_theta_delta_velocity.delta_theta[0],
		   &delta_theta_delta_velocity.delta_theta[1],
		   &delta_theta_delta_velocity.delta_theta[2],
		   &delta_theta_delta_velocity.delta_velocity[0],
		   &delta_theta_delta_velocity.delta_velocity[1],
		   &delta_theta_delta_velocity.delta_velocity[2]
		  ) != 7) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_real_time_hsi_results(const_buf_ref_t msg_buf, len_t msg_len,
				      RealTimeHsiResults &real_time_hsi_results)
{
	ErrorCode err = verify_message(msg_buf, msg_len, RealTimeHsiResults::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, RealTimeHsiResults::FORMAT,
		   &real_time_hsi_results.gain[0][0],
		   &real_time_hsi_results.gain[0][1],
		   &real_time_hsi_results.gain[0][2],
		   &real_time_hsi_results.gain[1][0],
		   &real_time_hsi_results.gain[1][1],
		   &real_time_hsi_results.gain[1][2],
		   &real_time_hsi_results.gain[2][0],
		   &real_time_hsi_results.gain[2][1],
		   &real_time_hsi_results.gain[2][2],
		   &real_time_hsi_results.bias[0],
		   &real_time_hsi_results.bias[1],
		   &real_time_hsi_results.bias[2]
		  ) != 12) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

ErrorCode parse_heave_and_heave_rate(const_buf_ref_t msg_buf, len_t msg_len,
				     HeaveAndHeaveRate &heave_and_heave_rate)
{
	ErrorCode err = verify_message(msg_buf, msg_len, HeaveAndHeaveRate::ID);

	if (err != ErrorCode::OK) {
		return err;
	}

	// get the data after the register delim
	char *data_start = strchr(msg_buf + HEADER_SIZE, DELIM);

	if (!data_start || data_start - msg_buf + 1 >= msg_len) {
		return ErrorCode::InvalidParameter;
	}

	if (sscanf(data_start + 1, HeaveAndHeaveRate::FORMAT,
		   &heave_and_heave_rate.heave,
		   &heave_and_heave_rate.heave_rate,
		   &heave_and_heave_rate.delayed_heave
		  ) != 3) {
		return ErrorCode::InvalidParameter;
	}

	return ErrorCode::OK;
}

} // namespace msg

} // namespace vn
