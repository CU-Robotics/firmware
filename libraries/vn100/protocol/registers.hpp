/**
 * @file registers.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines the register information for the VectorNav VN100. This includes structs and supporting enums for each of the registers.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#pragma once

#include "protocol.hpp"

// registers follow this structure:
// - ID: register ID, static constant
// - SIZE: size in bytes, static constant
// - HEADER: message header for the register/command, static constant
// - FORMAT: format string for parsing the register data, static constant
//   - %s for strings
//   - %lu for unsigned long (uint32_t)
//   - %hhu for unsigned char (uint8_t)
//   - %hd for signed short (int16_t)
//   - %f for float
//   - %lf for double
// - data: actual data of the register, defined as struct members

namespace vn
{

// commands

struct WriteSettings {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER = msg::MessageType::WriteSettings; // message header for write settings
	// no args
};

struct RestoreFactorySettings {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER =
		msg::MessageType::RestoreFactorySettings; // message header for restore factory settings
	// no args
};

struct Reset {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER = msg::MessageType::Reset; // message header for reset
	// no args
};

struct FirmwareUpdate {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER = msg::MessageType::FirmwareUpdate; // message header for firmware update
	// no args
	// NOTE: not supported
};

enum class DisturbanceState : uint8_t {
	NotPresent = 0u,	// disturbance not present
	Present = 1u,		// disturbance present
};

struct KnownMagneticDisturbance {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER =
		msg::MessageType::KnownMagneticDisturbance; // message header for known magnetic disturbance
	static constexpr const char *FORMAT = "%hhu"; // format string for parsing

	DisturbanceState state = DisturbanceState::NotPresent; // disturbance state
};

struct KnownAccelerationDisturbance {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER =
		msg::MessageType::KnownAccelerationDisturbance; // message header for known acceleration disturbance
	static constexpr const char *FORMAT = "%hhu"; // format string for parsing

	DisturbanceState state = DisturbanceState::NotPresent; // disturbance state
};

struct AsyncOutputEnable {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER =
		msg::MessageType::AsyncOutputEnable; // message header for async output enable
	static constexpr const char *FORMAT = "%hhu"; // format string for parsing

	uint8_t enable = 0u; // enable async output
};

struct SetGyroBias {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER = msg::MessageType::SetGyroBias; // message header for set gyro bias
	// no args
};

struct PollBinaryOutputMessage {
	static constexpr uint8_t ID = 255u; // register ID (not used)
	static constexpr msg::MessageType HEADER =
		msg::MessageType::PollBinaryOutputMessage; // message header for poll binary output message
	// NOTE: not supported
};


// 3 CONFIGURATION REGISTERS

// 3.2.1 User Tag

struct UserTag {
	static constexpr uint8_t ID = 0u; // register ID
	static constexpr uint8_t SIZE = 20u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%19s"; // format string for parsing

	char tag[20] = {0};	// user defined tag register. truncated to 20 characters
};

// 3.2.2 Baud Rate

enum class BaudrateSetting : uint32_t {
	BAUD_9600 = 9600u,
	BAUD_19200 = 19200u,
	BAUD_38400 = 38400u,
	BAUD_57600 = 57600u,
	BAUD_115200 = 115200u,
	BAUD_128000 = 128000u,
	BAUD_230400 = 230400u,
	BAUD_460800 = 460800u,
	BAUD_921600 = 921600u,
};

struct Baudrate {
	static constexpr uint8_t ID = 5u; // register ID
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%lu"; // format string for parsing

	BaudrateSetting baudrate = BaudrateSetting::BAUD_115200; // baudrate register
};

// 3.2.3 Async Data Output Type
enum class Ador : uint32_t {
	OFF = 0u,		// asynchronous output turned off
	YPR = 1u,		// yaw, pitch, roll: reg 8
	QTN = 2u,		// quaternion: reg 9
	QMR = 8u,		// quaternion, magnetic, acceleration and angular rate: reg 15
	MAG = 10u,		// magnetic: reg 17
	ACC = 11u,		// acceleration: reg 18
	GYR = 12u,		// angular rate: reg 19
	MAR = 13u,		// magnetic, acceleration and angular rate: reg 20
	YMR = 14u,		// yaw, pitch, roll, magnetic, acceleration and angular rate: reg 27
	YBA = 16u,		// yaw, pitch, body true acceleration and angular rate: reg 239
	YIA = 17u,		// yaw, pitch, inertial true acceleration and angular rate: reg 240
	IMU = 19u,		// imu: reg 54
	DTV = 30u,		// delta theta and delta velocity: reg 80
	HVE = 34u,		// heave: reg 115
};

struct AsyncDataOutputType {
	static constexpr uint8_t ID = 6u; // register ID
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%lu"; // format string for parsing

	Ador ador = Ador::YMR; // asynchronous data output type register
};

// 3.2.4 Async Data Output Freq
enum class Adof : uint32_t {
	HZ_0 = 0u,	// asynchronous output turned off
	HZ_1 = 1u,	// 1 Hz
	HZ_2 = 2u,	// 2 Hz
	HZ_4 = 4u,	// 4 Hz
	HZ_5 = 5u,	// 5 Hz
	HZ_10 = 10u,	// 10 Hz
	HZ_20 = 20u,	// 20 Hz
	HZ_25 = 25u,	// 25 Hz
	HZ_40 = 40u,	// 40 Hz
	HZ_50 = 50u,	// 50 Hz
	HZ_100 = 100u,	// 100 Hz
	HZ_200 = 200u,	// 200 Hz
};

struct AsyncDataOutputFreq {
	static constexpr uint8_t ID = 7u; // register ID
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%lu"; // format string for parsing

	Adof adof = Adof::HZ_40; // asynchronous data output frequency register
};

// 3.2.5 Communication Protocol Control
enum class AsciiAppendCount : uint8_t {
	None = 0u,	// off
	SyncInCount = 1u,	// SyncIn Counter
	SyncInTime = 2u,	// SyncIn Time
	SyncOutCount = 3u,	// SyncOut Counter
};

enum class AsciiAppendStatus : uint8_t {
	None = 0u,	// do not append any statuses
	Ahrs = 1u,	// append AHRS status
	Imu = 3u,	// append IMU status
};

enum class SpiAppendCount : uint8_t {
	None = 0u,	// off
	SyncInCount = 1u,	// SyncIn Counter
	SyncInTime = 2u,	// SyncIn Time
	SyncOutCount = 3u,	// SyncOut Counter
};

enum class SpiAppendStatus : uint8_t {
	None = 0u,	// do not append any statuses
	Ahrs = 1u,	// append AHRS status
	Imu = 3u,	// append IMU status
};

enum class AsciiChecksum : uint8_t {
	Checksum8bit = 1u,	// 8-bit checksum
	Crc16bit = 3u,		// 16-bit CRC
};

enum class SpiChecksum : uint8_t {
	Off = 0u,		// off
	Checksum8bit = 1u,	// 8-bit checksum
	Crc16bit = 3u,		// 16-bit CRC
};

enum class ErrorMode : uint8_t {
	Ignore = 0u,	// ignore errors
	SendError = 1u,	// send error message
	AdorOff = 2u 	// send error and set ADOR register to OFF
};

struct CommunicationProtocolControl {
	static constexpr uint8_t ID = 30u; // register ID
	static constexpr uint8_t SIZE = 7u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu"; // format string for parsing

	AsciiAppendCount ascii_append_count = AsciiAppendCount::None; // ASCII append count
	AsciiAppendStatus ascii_append_status = AsciiAppendStatus::None; // ASCII append status
	SpiAppendCount spi_append_count = SpiAppendCount::None; // SPI append count
	SpiAppendStatus spi_append_status = SpiAppendStatus::None; // SPI append status
	AsciiChecksum ascii_checksum = AsciiChecksum::Checksum8bit; // ASCII checksum type
	SpiChecksum spi_checksum = SpiChecksum::Off; // SPI checksum type
	ErrorMode error_mode = ErrorMode::SendError; // error mode
};

// 3.2.6 Synchronization Control
enum class SyncInMode : uint8_t {
	Disable = 0u,	// disable SyncIn events
	Count = 3u,	// count SyncIn events only
	ImuSample = 4u,	// sample IMU data on SyncIn events
	AsyncAll = 5u,	// output all configured asynchronous data on SyncIn events
	Async0 = 6u,	// output only asynchronous data configured with 0 rate on SyncIn events
};

enum class SyncInEdge : uint8_t {
	Rising = 0u,	// rising edge
	Falling = 1u,	// falling edge
};

enum class SyncOutMode : uint8_t {
	None = 0u,	// no SyncOut events
	ImuStart = 1u,	// Trigger at start of IMU sampling
	ImuReady = 2u,	// Trigger when IMU data is ready
	NavReady = 3u,	// Trigger when navigation data is ready
};

enum class SyncOutPolarity : uint8_t {
	NegativePulse = 0u,	// idle high, negative pulse on trigger
	PositivePulse = 1u,	// idle low, positive pulse on trigger
};

struct SyncControl {
	static constexpr uint8_t ID = 32u; // register ID
	static constexpr uint8_t SIZE = 20u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%hhu,%hu,%lu,%hhu,%hhu,%hu,%lu,%lu"; // format string for parsing

	SyncInMode sync_in_mode = SyncInMode::Count; // SyncIn mode
	SyncInEdge sync_in_edge = SyncInEdge::Rising; // SyncIn edge
	uint16_t sync_in_skip_factor = 0u; // skip factor applied to the input required for a SyncIn event
	uint32_t reserved1 = 0u; // reserved, must be set to 0
	SyncOutMode sync_out_mode = SyncOutMode::NavReady; // SyncOut mode
	SyncOutPolarity sync_out_polarity = SyncOutPolarity::NegativePulse; // SyncOut polarity
	uint16_t sync_out_skip_factor = 0u; // skip factor applied to the output required for a SyncOut event
	uint32_t sync_out_pulse_width = 500000u; // sync out signal pulse width in nanoseconds
	uint32_t reserved2 = 0u; // reserved, must be set to 0
};

// 3.2.7 Legacy Compatibility Settings
struct LegacyCompatibilitySettings {
	static constexpr uint8_t ID = 206u; // register ID
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%hhu,%hhu,%hhu"; // format string for parsing

	uint8_t reserved1 = 0u; // reserved, must be set to 0
	uint8_t reserved2 = 0u; // reserved, must be set to 0
	uint8_t imu_legacy =
		false; // true: calibrated temperature from pressure sensor, false: calibrated temperature from microcontroller
	uint8_t hw_legacy = false; // true: revert pin io to legacy behavior, false: use current firmware behavior
};

// 3.2.8 Binary Output Message Configuration #1
enum class AsyncMode : uint16_t {
	Disabled = 0u,	// asynchronous output disabled
	Serial1 = 1u,	// asynchronous output on serial port 1
	Serial2 = 2u,	// asynchronous output on serial port 2
	Both = 3u,	// asynchronous output on both serial ports
};

struct BinaryOutputMessageConfig1 {
	static constexpr uint8_t ID = 75u; // register ID
	static constexpr uint8_t SIZE = 7u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hu,%hu,%hhx,%hx,%hx,%hx,%hx"; // format string for parsing

	AsyncMode async_mode = AsyncMode::Disabled; // asynchronous output mode
	uint16_t rate_divisor = 0u; // rate divisor for the asynchronous output, 0 means no output

	bin::BinaryMessage config{};
};

// 3.2.9 Binary Output Message Configuration #2
struct BinaryOutputMessageConfig2 {
	static constexpr uint8_t ID = 76u; // register ID
	static constexpr uint8_t SIZE = 7u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hu,%hu,%hhx,%hx,%hx,%hx,%hx"; // format string for parsing

	AsyncMode async_mode = AsyncMode::Disabled; // asynchronous output mode
	uint16_t rate_divisor = 0u; // rate divisor for the asynchronous output, 0 means no output

	bin::BinaryMessage config{};
};

// 3.2.10 Binary Output Message Configuration #3
struct BinaryOutputMessageConfig3 {
	static constexpr uint8_t ID = 77u; // register ID
	static constexpr uint8_t SIZE = 7u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hu,%hu,%hhx,%hx,%hx,%hx,%hx"; // format string for parsing

	AsyncMode async_mode = AsyncMode::Disabled; // asynchronous output mode
	uint16_t rate_divisor = 0u; // rate divisor for the asynchronous output, 0 means no output

	bin::BinaryMessage config{};
};

// 3.3.1 Magnetic and Gravity Reference Vectors
struct MagneticGravityReference {
	static constexpr uint8_t ID = 21u; // register ID
	static constexpr uint8_t SIZE = 24u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f"; // format string for parsing

	float mag_ref[3] = {0.234f, 0.0f, 0.4212f}; // magnetic reference vector (NED)
	float grav_ref[3] = {0.0f, 0.0f, -9.79375f}; // gravity reference vector (NED)
};

// 3.3.2 VPE Basic Control
enum class HeadingMode : uint8_t {
	Absolute = 0u,	// absolute heading
	Relative = 1u,	// relative heading
	Indoor = 2u,	// indoor heading
};

enum class FilteringMode : uint8_t {
	Unfiltered = 0u,		// pass unfiltered measurements to the Kalman filter
	AdaptivelyFiltered = 1u,	// pass adaptively filtered measurements to the Kalman filter
};

enum class TuningMode : uint8_t {
	Static = 0u,		// use static tuning parameters
	Adaptive = 1u,		// use adaptive tuning parameters
};

struct VpeBasicControl {
	static constexpr uint8_t ID = 35u; // register ID
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%hhu,%hhu,%hhu"; // format string for parsing

	uint8_t reserved = 1u; // reserved, must be set to 1
	HeadingMode heading_mode = HeadingMode::Relative; // heading mode
	FilteringMode filtering_mode = FilteringMode::AdaptivelyFiltered; // filtering mode
	TuningMode tuning_mode = TuningMode::Adaptive; // tuning mode
};

// 3.3.3 VPE Magnetometer Basic Tuning
struct VpeMagnetometerBasicTuning {
	static constexpr uint8_t ID = 36u; // register ID
	static constexpr uint8_t SIZE = 36u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float base_tuning[3] = {4.0f, 4.0f, 4.0f}; // base tuning parameters for the magnetometer (x, y, z)-axis
	float adaptive_tuning[3] = {5.0f, 5.0f, 5.0f}; // adaptive tuning parameters for the magnetometer (x, y, z)-axis
	float adaptive_filtering[3] = {5.5f, 5.5f, 5.5f}; // adaptive filtering parameters for the magnetometer (x, y, z)-axis
};

// 3.3.4 VPE Accelerometer Basic Tuning
struct VpeAccelerometerBasicTuning {
	static constexpr uint8_t ID = 38u; // register ID
	static constexpr uint8_t SIZE = 36u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float base_tuning[3] = {6.0f, 6.0f, 6.0f}; // base tuning parameters for the accelerometer (x, y, z)-axis
	float adaptive_tuning[3] = {3.0f, 3.0f, 3.0f}; // adaptive tuning parameters for the accelerometer (x, y, z)-axis
	float adaptive_filtering[3] = {5.0f, 5.0f, 5.0f}; // adaptive filtering parameters for the accelerometer (x, y, z)-axis
};

// 3.3.5 Filter Startup Gyro Bias
struct FilterStartupGyroBias {
	static constexpr uint8_t ID = 43u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f"; // format string for parsing

	float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; // filter startup gyro bias (x, y, z)-axis
};

// 3.4.1 Magnetometer calibration
struct MagnetometerCalibration {
	static constexpr uint8_t ID = 23u; // register ID
	static constexpr uint8_t SIZE = 48u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float mag_gain[3][3] = { // magnetometer gain matrix
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	};
	float mag_bias[3] = {0.0f, 0.0f, 0.0f}; // magnetometer bias (x, y, z)-axis in Gauss
};

// 3.4.2 Accelerometer calibration
struct AccelerometerCalibration {
	static constexpr uint8_t ID = 25u; // register ID
	static constexpr uint8_t SIZE = 48u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float acc_gain[3][3] = { // accelerometer gain matrix
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	};
	float acc_bias[3] = {0.0f, 0.0f, 0.0f}; // accelerometer bias (x, y, z)-axis in m/s^2
};

// 3.4.3 Reference Frame Rotation
struct ReferenceFrameRotation {
	static constexpr uint8_t ID = 26u; // register ID
	static constexpr uint8_t SIZE = 36u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float rotation_matrix[3][3] = { // cosine rotation matrix
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	};
};

// 3.4.4 Delta Theta and Delta Velocity Configuration
enum class IntegrationFrame : uint8_t {
	Body = 0u,	// body frame
	NED = 1u,	// North-East-Down (NED) frame
};

struct DeltaThetaDeltaVelocityConfig {
	static constexpr uint8_t ID = 82u; // register ID
	static constexpr uint8_t SIZE = 6u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%hhu,%hhu,%hhu,%hu"; // format string for parsing

	IntegrationFrame integration_frame = IntegrationFrame::Body; // integration frame for delta theta and delta velocity
	uint8_t gyro_compensation =
		false; // true: use bias-compensated gyro measurements, false: use uncompensated gyro measurements
	uint8_t acc_compensation =
		false; // true: use gravity removed accelerometer measurements, false: use uncompensated accelerometer measurements
	uint8_t reserved1 = 0u; // reserved, must be set to 0
	uint16_t reserved2 = 0u; // reserved, must be set to 0
};

// 3.4.5 Gyro Calibration
struct GyroCalibration {
	static constexpr uint8_t ID = 84u; // register ID
	static constexpr uint8_t SIZE = 48u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float gyro_gain[3][3] = { // gyro gain matrix
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	};
	float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; // gyro bias (x, y, z)-axis in rad/s
};

// 3.4.6 IMU Filtering Configuration
enum class FilterMode : uint8_t {
	None = 0u,	// no filtering
	Uncomp = 1u,	// filter uncompensated data
	Comp = 2u,	// filter compensated data
	Both = 3u,	// filter both uncompensated and compensated data
};

struct ImuFilteringConfig {
	static constexpr uint8_t ID = 85u; // register ID
	static constexpr uint8_t SIZE = 15u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hu,%hu,%hu,%hu,%hu,%hhu,%hhu,%hhu,%hhu,%hhu"; // format string for parsing

	uint16_t mag_window_size = 0u; // previous measurements used for averaging magnetic measurements
	uint16_t acc_window_size = 4u; // previous measurements used for averaging accelerometer
	uint16_t gyro_window_size = 4u; // previous measurements used for averaging gyro measurements
	uint16_t temp_window_size = 4u; // previous measurements used for averaging temperature measurements
	uint16_t pres_window_size = 0u; // previous measurements used for averaging pressure measurements
	FilterMode mag_filter_mode = FilterMode::None; // filter mode for magnetic measurements
	FilterMode acc_filter_mode = FilterMode::Both; // filter mode for accelerometer measurements
	FilterMode gyro_filter_mode = FilterMode::Both; // filter mode for gyro measurements
	FilterMode temp_filter_mode = FilterMode::Both; // filter mode for temperature measurements
	FilterMode pres_filter_mode = FilterMode::None; // filter mode for pressure measurements
	uint8_t padding = 0u; // added for struct alignment. not part of any protocol
};

// 3.5.1 Real-Time HSI Control
enum class HsiMode : uint8_t {
	Off = 0u,	// HSI off
	Run = 1u, 	// Run the HSI
	Reset = 2u,	// Reset the HSI solution
};

enum class ApplyCompensation : uint8_t {
	Disable = 1u, // HSI estimate not applied to magnetic measurements
	Enable = 3u, // HSI estimate applied to magnetic measurements
};

struct RealTimeHsiControl {
	static constexpr uint8_t ID = 44u; // register ID
	static constexpr uint8_t SIZE = 3u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%hhu,%hhu"; // format string for parsing

	HsiMode hsi_mode = HsiMode::Off; // HSI mode
	ApplyCompensation apply_compensation = ApplyCompensation::Disable; // apply HSI compensation to magnetic measurements
	uint8_t converge_rate = 5u; // convergence rate for the HSI solution [1 - 5] (1 = slowest, 5 = fastest)
};

// 3.6.1 Velocity Aiding Measurement
struct VelocityAidingMeasurement {
	static constexpr uint8_t ID = 50u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f"; // format string for parsing

	float velocity[3] = {0.0f, 0.0f, 0.0f}; // velocity aiding measurement (x, y, z)-axis in m/s
};

// 3.6.2 Velocity Aiding Control
struct VelocityAidingControl {
	static constexpr uint8_t ID = 51u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%f,%f"; // format string for parsing

	uint8_t enable = true; // true: enable velocity aiding, false: disable velocity aiding
	uint8_t padding[3] = {0, 0, 0}; // padding, used for SPI only
	float vel_uncert_tuning = 0.1f; // velocity uncertainty tuning
	float reserved = 0.01f; // reserved, must be set to 0.01f
};

// 3.7.1 Reference Model Configuration
struct ReferenceModelConfig {
	static constexpr uint8_t ID = 83u;
	static constexpr uint8_t SIZE = 40u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%hhu,%hhu,%hhu,%hhu,%lu,%f,%lf,%lf,%lf"; // format string for parsing

	uint8_t enable_mag_model = false; // true: enable magnetic model as reference, false: disable magnetic model
	uint8_t enable_gravity_model = false; // true: enable gravity model as reference, false: disable gravity model
	uint8_t reserved1 = 0u; // reserved, must be set to 0
	uint8_t reserved2 = 0u; // reserved, must be set to 0
	uint32_t recalc_threshold = 1000u; // max distance traveled before the reference model is recalculated
	float year = 0.0f; // year as a decimal value
	uint8_t padding[4] = {0, 0, 0, 0}; // padding, used for SPI only
	double latitude = 0.0; // latitude in degrees
	double longitude = 0.0; // longitude in degrees
	double altitude = 0.0; // altitude in meters
};

// 3.8.1 Heave Basic Configuration
struct HeaveBasicConfig {
	static constexpr uint8_t ID = 116u; // register ID
	static constexpr uint8_t SIZE = 28u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float initial_wave_period = 0.0f; // initial wave period in seconds
	float initial_wave_amplitude = 0.05f; // initial wave amplitude in meters
	float max_wave_period = 20.0f; // maximum wave period in seconds
	float min_wave_amplitude = 0.05f; // minimum wave amplitude in meters
	float delayed_heave_cutoff_freq = 0.08f; // delayed heave cutoff frequency in rad/s
	float heave_cutoff_freq = 0.0f; // heave cutoff frequency in rad/s
	float heave_rate_cutoff_freq = 0.0f; // heave rate cutoff frequency in rad/s
};

// 4 MEASUREMENT REGISTERS (read only)

// 4.2.1 Model
struct Model {
	static constexpr uint8_t ID = 1u; // register ID
	static constexpr uint8_t SIZE = 24u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%23s"; // format string for parsing

	char model[24] = {0}; // product model name
};

// 4.2.2 Hardware Version
struct HardwareVersion {
	static constexpr uint8_t ID = 2u; // register ID
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%lu"; // format string for parsing

	uint32_t hardware_version = 0u; // hardware version number
};

// 4.2.3 Serial Number
struct SerialNumber {
	static constexpr uint8_t ID = 3u; // register ID
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%lu"; // format string for parsing

	uint32_t serial_number = 0u; // serial number
};

// 4.2.4 Firmware Version
struct FirmwareVersion {
	static constexpr uint8_t ID = 4u; // register ID
	static constexpr uint8_t SIZE = 20u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%19s"; // format string for parsing

	char firmware_version[20] = {0}; // firmware version string
};

// 4.2.5 Synchronization Status
struct SyncStatus {
	static constexpr uint8_t ID = 33u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%lu,%lu,%lu"; // format string for parsing

	uint32_t sync_in_count = 0u; // number of SyncIn events
	uint32_t sync_in_time = 0u; // time of the last SyncIn event in microseconds
	uint32_t sync_out_count = 0u; // number of SyncOut events
};

// 4.3.1 Yaw Pitch Roll
struct YawPitchRoll {
	static constexpr uint8_t ID = 8u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f"; // format string for parsing

	float yaw = 0.0f; // yaw angle in radians
	float pitch = 0.0f; // pitch angle in radians
	float roll = 0.0f; // roll angle in radians
};

// 4.3.2 Quaternion
struct Quaternion {
	static constexpr uint8_t ID = 9u; // register ID
	static constexpr uint8_t SIZE = 16u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f"; // format string for parsing

	float quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // quaternion (x, y, z, scalar) components
};

// 4.3.3 Quaternion & Compensated IMU
struct QuaternionCompensatedImu {
	static constexpr uint8_t ID = 15u; // register ID
	static constexpr uint8_t SIZE = 52u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // quaternion (x, y, z, scalar) components
	float mag[3] = {0.0f, 0.0f, 0.0f}; // magnetic field vector (x, y, z)-axis in Gauss
	float acc[3] = {0.0f, 0.0f, 0.0f}; // accelerometer vector (x, y, z)-axis in m/s^2
	float gyro[3] = {0.0f, 0.0f, 0.0f}; // gyro vector (x, y, z)-axis in rad/s
};

// 4.3.4 Yaw-Pitch-Roll & Compensated IMU
struct YawPitchRollCompensatedImu {
	static constexpr uint8_t ID = 27u; // register ID
	static constexpr uint8_t SIZE = 48u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float yaw = 0.0f; // yaw angle in radians
	float pitch = 0.0f; // pitch angle in radians
	float roll = 0.0f; // roll angle in radians
	float mag[3] = {0.0f, 0.0f, 0.0f}; // magnetic field vector (x, y, z)-axis in Gauss
	float acc[3] = {0.0f, 0.0f, 0.0f}; // accelerometer vector (x, y, z)-axis in m/s^2
	float gyro[3] = {0.0f, 0.0f, 0.0f}; // gyro vector (x, y, z)-axis in rad/s
};

// 4.3.5 Yaw-Pitch-Roll, Linear Acceleration & Angular Rate
struct YawPitchRollLinearAccelGyro {
	static constexpr uint8_t ID = 239u; // register ID
	static constexpr uint8_t SIZE = 36u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float yaw = 0.0f; // yaw angle in radians
	float pitch = 0.0f; // pitch angle in radians
	float roll = 0.0f; // roll angle in radians
	float lin_accel[3] = {0.0f, 0.0f, 0.0f}; // linear acceleration vector (x, y, z)-axis in m/s^2
	float gyro[3] = {0.0f, 0.0f, 0.0f}; // angular rate vector (x, y, z)-axis in rad/s
};

// 4.3.6 Yaw-Pitch-Roll, Linear Inertial Acceleration & Angular Rate
struct YawPitchRollLinearInertialAccelGyro {
	static constexpr uint8_t ID = 240u; // register ID
	static constexpr uint8_t SIZE = 36u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float yaw = 0.0f; // yaw angle in radians
	float pitch = 0.0f; // pitch angle in radians
	float roll = 0.0f; // roll angle in radians
	float lin_accel[3] = {0.0f, 0.0f, 0.0f}; // linear inertial acceleration vector (x, y, z)-axis in m/s^2
	float gyro[3] = {0.0f, 0.0f, 0.0f}; // angular rate vector (x, y, z)-axis in rad/s
};

// 4.4.1 Compensated Magnetometer
struct CompensatedMagnetometer {
	static constexpr uint8_t ID = 17u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f"; // format string for parsing

	float mag[3] = {0.0f, 0.0f, 0.0f}; // compensated magnetic field vector (x, y, z)-axis in Gauss
};

// 4.4.2 Compensated Accelerometer
struct CompensatedAccelerometer {
	static constexpr uint8_t ID = 18u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f"; // format string for parsing

	float acc[3] = {0.0f, 0.0f, 0.0f}; // compensated accelerometer vector (x, y, z)-axis in m/s^2
};

// 4.4.3 Compensated Gyro
struct CompensatedGyro {
	static constexpr uint8_t ID = 19u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f"; // format string for parsing

	float gyro[3] = {0.0f, 0.0f, 0.0f}; // compensated gyro vector (x, y, z)-axis in rad/s
};

// 4.4.4 Compensated IMU
struct CompensatedImu {
	static constexpr uint8_t ID = 20u; // register ID
	static constexpr uint8_t SIZE = 36u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float mag[3] = {0.0f, 0.0f, 0.0f}; // compensated magnetic field vector (x, y, z)-axis in Gauss
	float acc[3] = {0.0f, 0.0f, 0.0f}; // compensated accelerometer vector (x, y, z)-axis in m/s^2
	float gyro[3] = {0.0f, 0.0f, 0.0f}; // compensated gyro vector (x, y, z)-axis in rad/s
};

// 4.4.5 IMU Measurements
struct ImuMeasurements {
	static constexpr uint8_t ID = 54u; // register ID
	static constexpr uint8_t SIZE = 44u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float uncomp_mag[3] = {0.0f, 0.0f, 0.0f}; // uncompensated magnetic field vector (x, y, z)-axis in Gauss
	float uncomp_acc[3] = {0.0f, 0.0f, 0.0f}; // uncompensated accelerometer vector (x, y, z)-axis in m/s^2
	float uncomp_gyro[3] = {0.0f, 0.0f, 0.0f}; // uncompensated gyro vector (x, y, z)-axis in rad/s
	float temp = 0.0f; // temperature in degrees Celsius
	float pres = 0.0f; // pressure in kPascals
};

// 4.4.6 Delta Theta and Delta Velocity
struct DeltaThetaDeltaVelocity {
	static constexpr uint8_t ID = 80u; // register ID
	static constexpr uint8_t SIZE = 28u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float delta_time = 0.0f; // delta time in seconds
	float delta_theta[3] = {0.0f, 0.0f, 0.0f}; // delta theta vector (x, y, z)-axis in degrees
	float delta_velocity[3] = {0.0f, 0.0f, 0.0f}; // delta velocity vector (x, y, z)-axis in m/s
};

// 4.5.1 Real-Time HSI Results
struct RealTimeHsiResults {
	static constexpr uint8_t ID = 47u; // register ID
	static constexpr uint8_t SIZE = 48u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"; // format string for parsing

	float gain[3][3] = { // HSI gain matrix
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	};
	float bias[3] = {0.0f, 0.0f, 0.0f}; // HSI bias (x, y, z)-axis in Gauss
};

// 4.6.1 Heave and Heave Rate
struct HeaveAndHeaveRate {
	static constexpr uint8_t ID = 115u; // register ID
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr msg::MessageType HEADER = msg::MessageType::Unknown; // command header (not used)
	static constexpr const char *FORMAT = "%f,%f,%f"; // format string for parsing

	float heave = 0.0f; // heave in meters
	float heave_rate = 0.0f; // heave rate in meters per second
	float delayed_heave = 0.0f; // delayed heave in meters
};

namespace bin
{

// 2.2 Common Group

// copy of Time Group TimeStartup
struct CommonTimeStartup {
	static constexpr uint8_t SIZE = 8u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 0u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	uint64_t time_startup = 0u; // time since startup in nanoseconds
};

// copy of Time Group TypeSyncIn
struct CommonTypeSyncIn {
	static constexpr uint8_t SIZE = 8u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 2u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	uint64_t time_sync_in = 0u; // time of the SyncIn event in nanoseconds
};

// copy of Attitude Group Ypr
struct CommonYpr {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 3u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	float yaw = 0.0f; // yaw angle in degrees
	float pitch = 0.0f; // pitch angle in degrees
	float roll = 0.0f; // roll angle in degrees
};

// copy of Attitude Group Quaternion
struct CommonQuaternion {
	static constexpr uint8_t SIZE = 16u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 4u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	float quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // quaternion (x, y, z, scalar) components
};

// copy of IMU Group AngularRate
struct CommonAngularRate {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 5u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	float angular_rate[3] = {0.0f, 0.0f, 0.0f}; // angular rate (x, y, z)-axis in rad/s
};

// copy of IMU Group Accel
struct CommonAccel {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 8u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	float acc[3] = {0.0f, 0.0f, 0.0f}; // accelerometer vector (x, y, z)-axis in m/s^2
};

// concatenation of IMU Group UncompAccel, UncompGyro
struct CommonImu {
	static constexpr uint8_t SIZE = 24u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 9u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	float uncomp_acc[3] = {0.0f, 0.0f, 0.0f}; // uncompensated accelerometer vector (x, y, z)-axis in m/s^2
	float uncomp_gyro[3] = {0.0f, 0.0f, 0.0f}; // uncompensated gyro vector (x, y, z)-axis in rad/s
};

// concatenation of IMU Group Mag, Temperature, Pressure
struct CommonMagPres {
	static constexpr uint8_t SIZE = 20u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 10u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	float mag[3] = {0.0f, 0.0f, 0.0f}; // magnetic field vector (x, y, z)-axis in Gauss
	float temperature = 0.0f; // temperature in degrees Celsius
	float pressure = 0.0f; // pressure in kPascals
};

// concatenation of IMU Group DeltaTheta, DeltaVel
struct CommonDeltas {
	static constexpr uint8_t SIZE = 28u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 11u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	float delta_time = 0.0f; // delta time in seconds
	float delta_theta[3] = {0.0f, 0.0f, 0.0f}; // delta theta vector (x, y, z)-axis in degrees
	float delta_velocity[3] = {0.0f, 0.0f, 0.0f}; // delta velocity vector (x, y, z)-axis in m/s
};

// copy of Time Group SyncInCnt
struct CommonSyncInCnt {
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 13u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::CommonGroup;

	uint32_t sync_in_count = 0u; // number of SyncIn events
};

// 2.3 Time Group

// 2.3.1 TimeStartup
struct TimeStartup {
	static constexpr uint8_t SIZE = 8u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 0u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::TimeGroup;

	uint64_t time_startup = 0u; // time since startup in nanoseconds
};

// 2.3.2 TimeSyncIn
struct TimeSyncIn {
	static constexpr uint8_t SIZE = 8u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 4u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::TimeGroup;

	uint64_t time_sync_in = 0u; // time of the SyncIn event in nanoseconds
};

// 2.3.3 SyncInCnt
struct SyncInCnt {
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 7u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::TimeGroup;

	uint32_t sync_in_count = 0u; // number of SyncIn events
};

// 2.3.4 SyncOutCnt
struct SyncOutCnt {
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 8u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::TimeGroup;

	uint32_t sync_out_count = 0u; // number of SyncOut events
};

// 2.4 IMU Group

enum class SensorStatus : uint8_t {
	NominalUpdated = 0u, // nominal sensor status, updated
	NominalNotUpdated = 1u, // nominal sensor status, not updated due to lower sampling rates
	Saturated = 2u, // sensor data saturated
	Failed = 3u, // sensor failed and data is not valid
};

struct ImuStatus {
	static constexpr uint8_t SIZE = 2u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 0u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	SensorStatus gyro_status : 2; // gyro sensor status
	SensorStatus acc_status : 2; // accelerometer sensor status
	SensorStatus mag_status : 2; // magnetometer sensor status
	SensorStatus pres_temp_status : 2; // pressure and temperature sensor status
	uint16_t reserved : 2;
};

// 2.4.2 UncompMag
struct UncompMag {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 1u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float uncomp_mag[3] = {0.0f, 0.0f, 0.0f}; // uncompensated magnetic field vector (x, y, z)-axis in Gauss
};

// 2.4.3 UncompAcc
struct UncompAcc {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 2u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float uncomp_acc[3] = {0.0f, 0.0f, 0.0f}; // uncompensated accelerometer vector (x, y, z)-axis in m/s^2
};

// 2.4.4 UncompGyro
struct UncompGyro {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 3u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float uncomp_gyro[3] = {0.0f, 0.0f, 0.0f}; // uncompensated gyro vector (x, y, z)-axis in rad/s
};

// 2.4.5 Temperature
struct Temperature {
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 4u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float temperature = 0.0f; // temperature in degrees Celsius
};

// 2.4.6 Pressure
struct Pressure {
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 5u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float pressure = 0.0f; // pressure in kPascals
};

// 2.4.7 DeltaTheta
struct DeltaTheta {
	static constexpr uint8_t SIZE = 16u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 6u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float delta_time = 0.0f; // delta time in seconds
	float delta_theta[3] = {0.0f, 0.0f, 0.0f}; // delta theta vector (x, y, z)-axis in degrees
};

// 2.4.8 DeltaVel
struct DeltaVel {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 7u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float delta_velocity[3] = {0.0f, 0.0f, 0.0f}; // delta velocity vector (x, y, z)-axis in m/s
};

// 2.4.9 Mag
struct Mag {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 8u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float mag[3] = {0.0f, 0.0f, 0.0f}; // magnetic field vector (x, y, z)-axis in Gauss
};

// 2.4.10 Accel
struct Accel {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 9u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float acc[3] = {0.0f, 0.0f, 0.0f}; // accelerometer vector (x, y, z)-axis in m/s^2
};

// 2.4.11 AngularRate
struct AngularRate {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 10u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	float gyro[3] = {0.0f, 0.0f, 0.0f}; // gyroscope vector (x, y, z)-axis in rad/s
};

// 2.4.11 SensSat
struct SensSat {
	static constexpr uint8_t SIZE = 2u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 11u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::ImuGroup;

	uint16_t mag_x : 1; // magnetic field saturation status for x-axis
	uint16_t mag_y : 1; // magnetic field saturation status for y-axis
	uint16_t mag_z : 1; // magnetic field saturation status for z-axis
	uint16_t acc_x : 1; // accelerometer saturation status for x-axis
	uint16_t acc_y : 1; // accelerometer saturation status for y-axis
	uint16_t acc_z : 1; // accelerometer saturation status for z-axis
	uint16_t gyro_x : 1; // gyroscope saturation status for x-axis
	uint16_t gyro_y : 1; // gyroscope saturation status for y-axis
	uint16_t gyro_z : 1; // gyroscope saturation status for z-axis
	uint16_t pres : 1; // pressure sensor saturation status
};

// 2.5 Attitude Group

// 2.5.1 AhrsStatus
enum class AttitudeQuality : uint8_t {
	Excellent = 0u, // filter consistency high and attidue uncertainty low
	Good = 1u, // filter consistency medium to high and attitude uncertainty low to medium
	Bad = 2u, // poor filter consistency or high attitude uncertainty
	NotTracking = 3u // attitude solution cannot be relied upon
};

struct AhrsStatus {
	static constexpr uint8_t SIZE = 2u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 0u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	AttitudeQuality attitude_quality : 2; // attitude quality
	uint16_t gyro_saturation : 1; // at least one gyro axis is currently saturated
	uint16_t gyro_saturation_recovery : 1; // filter is in the process of recovering from a gyro saturation
	uint16_t mag_disturbance : 1; // level of magnetic dc disturbance detected
	uint16_t : 1;
	uint16_t mag_saturation : 1; // at least one magnetometer axis is currently saturated
	uint16_t accel_disturbance : 1; // level of accelerometer disturbance detected
	uint16_t : 1;
	uint16_t accel_saturation : 1; // at least one accelerometer axis is currently saturated
	uint16_t : 1;
	uint16_t known_mag_disturbance : 1; // a KMD is being reported by user and is currently being tuned out
	uint16_t known_accel_disturbance : 1; // an KAD is being reported by user and is currently being tuned out
	uint16_t : 3;
};

// 2.5.2 Ypr
struct Ypr {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 1u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float yaw = 0.0f; // yaw angle in degrees
	float pitch = 0.0f; // pitch angle in degrees
	float roll = 0.0f; // roll angle in degrees
};

// 2.5.3 Quaternion
struct Quaternion {
	static constexpr uint8_t SIZE = 16u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 2u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // quaternion (x, y, z, s)
};

// 2.5.4 Dcm
struct Dcm {
	static constexpr uint8_t SIZE = 36u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 3u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float dcm[3][3] = { // direction cosine matrix
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	};
};

// 2.5.5 MagNed
struct MagNed {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 4u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float mag_ned[3] = {0.0f, 0.0f, 0.0f}; // magnetic field vector in North-East-Down (NED) frame (x, y, z)-axis in Gauss
};

// 2.5.6 AccelNed
struct AccelNed {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 5u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float accel_ned[3] = {0.0f, 0.0f, 0.0f}; // accelerometer vector in North-East-Down (NED) frame (x, y, z)-axis in m/s^2
};

// 2.5.7 LinBodyAcc
struct LinBodyAcc {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 6u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float lin_body_acc[3] = {0.0f, 0.0f, 0.0f}; // linear body acceleration vector (x, y, z)-axis in m/s^2
};

// 2.5.8 LinAccelNed
struct LinAccelNed {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 7u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float lin_accel_ned[3] = {0.0f, 0.0f, 0.0f}; // linear acceleration vector in North-East-Down (NED) frame (x, y, z)-axis in m/s^2
};

// 2.5.9 YprU
struct YprU {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 8u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float yaw_u = 0.0f; // filter estimate uncertainty in degrees
	float pitch_u = 0.0f; // filter estimate uncertainty in degrees
	float roll_u = 0.0f; // filter estimate uncertainty in degrees
};

// 2.5.10 Heave
struct Heave {
	static constexpr uint8_t SIZE = 12u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 12u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float heave = 0.0f; // heave in meters
	float heave_rate = 0.0f; // heave rate in meters per second
	float delayed_heave = 0.0f; // delayed heave in meters
};

// 2.5.11 AttU
struct AttU {
	static constexpr uint8_t SIZE = 4u; // size in bytes
	static constexpr uint16_t TYPE = 1u << 13u; // bit offset in the message
	static constexpr uint8_t GROUP = vn::bin::AttitudeGroup;

	float att_u = 0.0f; // current attitude estimate uncertainty in degrees
};

} // namespace bin

} // namespace vn
