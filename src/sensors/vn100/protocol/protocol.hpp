/**
 * @file protocol.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines the various protocol information including message structure, CRC calculation, etc. for the VectorNav VN100.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#pragma once

#include <stdint.h>

namespace vn
{

namespace msg
{

// 1.1 UART SERIAL INTERFACE

// message delimiters
constexpr uint8_t START = '$';		// start of message
constexpr uint8_t END_DATA = '*';	// end of data, start of checksum
constexpr uint8_t END = '\n';		// end of checksum and message
constexpr uint8_t DELIM = ',';		// delimiter for data fields

// message structure
constexpr uint8_t HEADER_SIZE = 7;	// size of the header ($ + 5 characters for the message type + DELIM)
constexpr uint8_t CHECKSUM_SIZE = 2; 	// size of the checksum (two hex characters)

// maximum buffer sizes
constexpr uint32_t MAX_MESSAGE_SIZE = 256;	// maximum message size in bytes

// length of the message in bytes, can be negative on error
using len_t = int;
// reference to a message buffer type, used in read/write contexts
using buf_ref_t = char (&)[MAX_MESSAGE_SIZE];
// const buffer reference type for messages, used in read-only contexts
using const_buf_ref_t = const char (&)[MAX_MESSAGE_SIZE];
// buffer type for messages, used in parsing and creation functions
using buf_t = char [MAX_MESSAGE_SIZE];

// 1.3 COMMANDS

// message headers
constexpr const char *VNERR = "VNERR";	// error
constexpr const char *VNRRG = "VNRRG";	// read register
constexpr const char *VNWRG = "VNWRG";	// write register
constexpr const char *VNWNV = "VNWNV";	// write settings (non-volatile)
constexpr const char *VNRFS = "VNRFS";	// restore factory settings
constexpr const char *VNRST = "VNRST";	// reset
constexpr const char *VNFWU = "VNFWU";	// firmware update
constexpr const char *VNKMD = "VNKMD";	// known magnetic disturbance
constexpr const char *VNKAD = "VNKAD";	// known acceleration disturbance
constexpr const char *VNASY = "VNASY";	// async output enable
constexpr const char *VNSGB = "VNSGB";	// set gyro bias
constexpr const char *VNBOM = "VNBOM";	// poll binary output message

// message enum
enum class MessageType : uint8_t {
	Unknown = 0u,				// unknown message type
	Error = 1u,				// error message
	ReadRegister = 2u,			// read register message
	WriteRegister = 3u,			// write register message
	WriteSettings = 4u,			// write non-volatile settings message
	RestoreFactorySettings = 5u,		// restore factory settings message
	Reset = 6u,				// reset message
	FirmwareUpdate = 7u,			// firmware update message
	KnownMagneticDisturbance = 8u,		// set known magnetic disturbance message
	KnownAccelerationDisturbance = 9u,	// set known acceleration disturbance message
	AsyncOutputEnable = 10u,		// async output enable message
	SetGyroBias = 11u,			// set gyro bias message
	PollBinaryOutputMessage = 12u,		// poll binary output message
};

} // namespace msg

// 1.4 CHECKSUM

// the checksum can be ignored if needed
constexpr const char *CHECKSUM_IGNORE = "XX";

// calculates the 8-bit XOR checksum for the given byte sequence
inline void calculate_checksum(char hex[2], const uint8_t *data, const uint8_t length)
{
	static const char hex_digits[] = "0123456789ABCDEF";

	uint8_t cksum = 0;

	for (uint8_t i = 0; i < length; i++) {
		if (data[i] == vn::msg::START) { continue; } // skip start of message

		if (data[i] == vn::msg::END_DATA) { break; } // skip end of data

		cksum ^= data[i];
	}

	// convert to hex using digit map
	hex[0] = hex_digits[(cksum >> 4) & 0x0F];	// high nibble
	hex[1] = hex_digits[cksum & 0x0F];		// low nibble
}

// given a hex checksum, verify it against the data
inline bool verify_checksum(char hex[2], const uint8_t *data, const uint8_t length)
{
	char calculated[2];
	calculate_checksum(calculated, data, length);
	return (hex[0] == calculated[0] && hex[1] == calculated[1]);
}

// calculates the 16-bit CRC checksum for the given byte sequence
inline uint16_t calculate_crc(const uint8_t *data, const uint32_t length)
{
	uint16_t crc = 0;

	for (uint32_t i = 0; i < length; i++) {
		crc = (uint8_t)(crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (uint8_t)(crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0x00ff) << 5;
	}

	return crc;
}

// 1.5 SYSTEM ERROR CODES

enum class ErrorCode : uint8_t {
	// hardware error codes
	OK = 0x00,
	HardFault = 0x01,		// hard fault exception
	SerialBufferOverflow = 0x02, 	// serial buffer overflow
	InvalidChecksum = 0x03,		// received checksum was invalid
	InvalidCommand = 0x04,		// received command was invalid
	NotEnoughParameters = 0x05,	// not enough parameters for command
	TooManyParameters = 0x06,	// too many parameters for command
	InvalidParameter = 0x07,	// invalid parameter for command
	InvalidRegister = 0x08,		// requested register is invalid
	UnauthorizedAccess = 0x09,	// user does not have permission to write to this register
	WatchdogReset = 0x0A,		// internal 50ms watchdog timer reset
	OutputBufferOverflow = 0x0B,	// output buffer overflow
	InsufficientBaudRate = 0x0C,	// insufficient baudrate for requested async data output
	ErrorBufferOverflow = 0xFF,	// system error buffer overflowed

	// custom error codes
	FailedTransmission = 0xFE,	// failed to transmit message
	Timeout = 0xFC,			// operation timed out
};

inline const char *err_to_string(ErrorCode err)
{
	switch (err) {
	case ErrorCode::OK: return "OK";

	case ErrorCode::HardFault: return "HardFault";

	case ErrorCode::SerialBufferOverflow: return "SerialBufferOverflow";

	case ErrorCode::InvalidChecksum: return "InvalidChecksum";

	case ErrorCode::InvalidCommand: return "InvalidCommand";

	case ErrorCode::NotEnoughParameters: return "NotEnoughParameters";

	case ErrorCode::TooManyParameters: return "TooManyParameters";

	case ErrorCode::InvalidParameter: return "InvalidParameter";

	case ErrorCode::InvalidRegister: return "InvalidRegister";

	case ErrorCode::UnauthorizedAccess: return "UnauthorizedAccess";

	case ErrorCode::WatchdogReset: return "WatchdogReset";

	case ErrorCode::OutputBufferOverflow: return "OutputBufferOverflow";

	case ErrorCode::InsufficientBaudRate: return "InsufficientBaudRate";

	case ErrorCode::ErrorBufferOverflow: return "ErrorBufferOverflow";

	case ErrorCode::FailedTransmission: return "FailedTransmission";

	case ErrorCode::Timeout: return "Timeout";

	default: return "UnknownError";
	}
}

// 2 Binary Output Messages

namespace bin
{

// 2.1.3 Message Format

constexpr uint8_t START = 0xFAu;		// start of binary message
constexpr uint16_t CRC_GOOD = 0x0000u;		// crc result for a good crc

constexpr uint32_t MAX_BINARY_SIZE = 600u;	// maximum binary packet size in bytes

// reference to a binary buffer type, used in read/write contexts
using bin_buf_ref_t = uint8_t (&)[MAX_BINARY_SIZE];
// const buffer reference type for binary messages, used in read-only contexts
using const_bin_buf_ref_t = const uint8_t (&)[MAX_BINARY_SIZE];
// buffer type for binary output messages, used in parsing and creation functions
using bin_buf_t = uint8_t[MAX_BINARY_SIZE];

enum BinaryGroup : uint8_t {
	CommonGroup = 1u << 0u,
	TimeGroup = 1u << 1u,
	ImuGroup = 1u << 2u,
	AttitudeGroup = 1u << 4u,
};

enum class CommonTypes : uint16_t {
	TimeStartup = 1u << 0u,	// Copy of TimeTypes::TimeStartup
	TimeSyncIn = 1u << 2u,	// Copy of TimeTypes::TimeSyncIn
	Ypr = 1u << 3u,		// Copy of AttitudeTypes::Ypr
	Quaternion = 1u << 4u,	// Copy of AttitudeTypes::Quaternion
	AngularRate = 1u << 5u,	// Copy of ImuTypes::AngularRate
	Accel = 1u << 8u,	// Copy of ImuTypes::Accel
	Imu = 1u << 9u,		// Concatenation of ImuTypes::UncompAccel and ImuTypes::UncompGyro
	MagPres = 1u << 10u,	// Concatenation of ImuTypes::Mag, ImuTypes::Pressure, and ImuTypes::Temperature
	Deltas = 1u << 11u,	// Concatenation of ImuTypes::DeltaTheta and ImuTypes::DeltaVel
	SyncInCnt = 1u << 13u,	// Copy of TimeTypes::SyncInCnt
};

enum class TimeTypes : uint16_t {
	TimeStartup = 1u << 0u,	// time since system startup in nanoseconds
	TimeSyncIn = 1u << 4u,	// time of last sync in event in nanoseconds
	SyncInCnt = 1u << 7u,	// number of sync in events since system startup
	SyncOutCnt = 1u << 8u,	// number of sync out events since system startup
};

enum class ImuTypes : uint16_t {
	ImuStatus = 1u << 0u,	// Various status flags for IMU sensors
	UncompMag = 1u << 1u,	// IMU mag field in the body frame
	UncompAccel = 1u << 2u,	// IMU acceleration in the body frame
	UncompGyro = 1u << 3u,	// IMU angular rate in the body frame
	Temperature = 1u << 4u,	// IMU temperature
	Pressure = 1u << 5u,	// IMU pressure
	DeltaTheta = 1u << 6u,	// delta theta since last message
	DeltaVel = 1u << 7u,	// delta velocity since last message
	Mag = 1u << 8u,		// IMU compensated magnetic field in the body frame
	Accel = 1u << 9u,	// IMU compensated acceleration in the body frame
	AngularRate = 1u << 10u,// IMU compensated angular rate in the body frame
	SensSat = 1u << 11u,	// IMU sensor saturation status
};

enum class AttitudeTypes : uint16_t {
	AhrsStatus = 1u << 0u,	// AHRS status
	Ypr = 1u << 1u,		// attitude in yaw, pitch, roll with respect to the North-East-Down (NED) frame
	Quaternion = 1u << 2u, 	// attitude in quaternion with respect to the North-East-Down (NED) frame
	Dcm = 1u << 3u,		// attitude in direction cosine matrix (DCM) with respect to the North-East-Down (NED) frame
	MagNed = 1u << 4u,	// magnetic field in the North-East-Down (NED) frame
	AccelNed = 1u << 5u,	// acceleration (with gravity) in the North-East-Down (NED) frame
	LinBodyAcc = 1u << 6u,	// linear acceleration (without gravity) in the body frame
	LinAccelNed = 1u << 7u,	// linear acceleration (without gravity) in the North-East-Down (NED) frame
	YprU = 1u << 8u,	// estimated attitude uncertainty (1 Sigma) in degrees
	Heave = 1u << 12u,	// real-time heave and heave rate estimates
	AttU = 1u << 13u,	// estimated current attitude uncertainty (1 Sigma)
};

struct BinaryMessage {
	// the raw group bitfield
	uint8_t binary_group = 0u;
	// number of set groups in the binary_group
	uint8_t num_groups = 0u;

	// the raw group type bitfields per group. indexed by sequential group count
	uint16_t group_types[4] = {0, 0, 0, 0};
	// group indices, specifies the bit index of the group. indexed by sequential group count
	uint8_t group_indices[4] = {0, 0, 0, 0};

	// calculated size of the payload
	msg::len_t size = 0;
};

// callback type for binary message processing
using binary_callback_t = void (*)(const_bin_buf_ref_t buf, msg::len_t length, BinaryMessage &msg);

} // namespace bin

} // namespace vn
