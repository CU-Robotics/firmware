/**
 * @file msg_parsing.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines the message parsing functions for the VectorNav VN100. This includes functions to parse messages, validate checksums,
 *	      and handle message formatting. This also manages the conversion from raw message bytes to structs.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#pragma once

#include "registers.hpp"
#include "protocol.hpp"

namespace vn
{

namespace msg
{

/// @see msg_parsing.cpp for static inline message parsing helpers

/**
 * @brief Extracts the header information from a message buffer
 *
 * @param msg_buf The message buffer to extract the header from
 * @param msg_len The length of the message buffer
 * @param command The command type extracted from the header. If nullptr, the command type is ignored
 * @param reg_id The register ID extracted from the header. If nullptr, the register ID is ignored
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode extract_header(const_buf_ref_t msg_buf, len_t msg_len, MessageType *command, uint8_t *reg_id);

/**
 * @brief Verifies the integrity of a message by checking its header and checksum
 *
 * @param msg_buf The message buffer to verify
 * @param msg_len The length of the message buffer
 * @param reg_id The register ID extracted from the header. If not provided, it is not checked
 * @param command The command type expected in the header. If not provided, it is not checked
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode verify_message(const_buf_ref_t msg_buf, len_t msg_len, uint8_t reg_id = 255,
			 MessageType command = MessageType::Unknown);

/**
 * @brief Parses a complete message buffer into a WriteSettings struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param write_settings The WriteSettings struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_write_settings(const_buf_ref_t msg_buf, len_t msg_len, WriteSettings &write_settings);
/**
 * @brief Parses a complete message buffer into a RestoreFactorySettings struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param restore_factory_settings The RestoreFactorySettings struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_restore_factory_settings(const_buf_ref_t msg_buf, len_t msg_len,
		RestoreFactorySettings &restore_factory_settings);
/**
 * @brief Parses a complete message buffer into a Reset struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param reset The Reset struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_reset(const_buf_ref_t msg_buf, len_t msg_len, Reset &reset);
/**
 * @brief Parses a complete message buffer into a FirmwareUpdate struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param firmware_update The FirmwareUpdate struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_firmware_update(const_buf_ref_t msg_buf, len_t msg_len, FirmwareUpdate &firmware_update);
/**
 * @brief Parses a complete message buffer into a KnownMagneticDisturbance struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param mag_disturbance The KnownMagneticDisturbance struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_known_magnetic_disturbance(const_buf_ref_t msg_buf, len_t msg_len,
		KnownMagneticDisturbance &mag_disturbance);
/**
 * @brief Parses a complete message buffer into a KnownAccelerationDisturbance struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param accel_disturbance The KnownAccelerationDisturbance struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_known_acceleration_disturbance(const_buf_ref_t msg_buf, len_t msg_len,
		KnownAccelerationDisturbance &accel_disturbance);
/**
 * @brief Parses a complete message buffer into an AsyncOutputEnable struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param async_output_enable The AsyncOutputEnable struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_async_output_enable(const_buf_ref_t msg_buf, len_t msg_len, AsyncOutputEnable &async_output_enable);
/**
 * @brief Parses a complete message buffer into a SetGyroBias struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param set_gyro_bias The SetGyroBias struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_set_gyro_bias(const_buf_ref_t msg_buf, len_t msg_len, SetGyroBias &set_gyro_bias);
/**
 * @brief Parses a complete message buffer into a PollBinaryOutputMessage struct
 *
 * @param msg_buf The message buffer to parse
 * @param msg_len The length of the message buffer
 * @param poll_binary_output_message The PollBinaryOutputMessage struct to populate
 * @return ErrorCode indicating the result of the operation
 */
ErrorCode parse_poll_binary_output_message(const_buf_ref_t msg_buf, len_t msg_len,
		PollBinaryOutputMessage &poll_binary_output_message);

/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param user_tag The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_user_tag(const_buf_ref_t msg_buf, len_t msg_len, UserTag &user_tag);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param baudrate The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_baudrate(const_buf_ref_t msg_buf, len_t msg_len, Baudrate &baudrate);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param async_data_output_type The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_async_data_output_type(const_buf_ref_t msg_buf, len_t msg_len,
				       AsyncDataOutputType &async_data_output_type);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param async_data_output_freq The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_async_data_output_freq(const_buf_ref_t msg_buf, len_t msg_len,
				       AsyncDataOutputFreq &async_data_output_freq);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param comm_protocol_control The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_communication_protocol_control(const_buf_ref_t msg_buf, len_t msg_len,
		CommunicationProtocolControl &comm_protocol_control);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param sync_control The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_sync_control(const_buf_ref_t msg_buf, len_t msg_len, SyncControl &sync_control);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param legacy_settings The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_legacy_compatibility_settings(const_buf_ref_t msg_buf, len_t msg_len,
		LegacyCompatibilitySettings &legacy_settings);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param binary_output_msg_config The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_binary_output_message_config_1(const_buf_ref_t msg_buf, len_t msg_len,
		BinaryOutputMessageConfig1 &binary_output_msg_config);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param binary_output_msg_config The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_binary_output_message_config_2(const_buf_ref_t msg_buf, len_t msg_len,
		BinaryOutputMessageConfig2 &binary_output_msg_config);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param binary_output_msg_config The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_binary_output_message_config_3(const_buf_ref_t msg_buf, len_t msg_len,
		BinaryOutputMessageConfig3 &binary_output_msg_config);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param mag_grav_ref The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_magnetic_gravity_reference(const_buf_ref_t msg_buf, len_t msg_len,
		MagneticGravityReference &mag_grav_ref);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param vpe_basic_control The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_vpe_basic_control(const_buf_ref_t msg_buf, len_t msg_len, VpeBasicControl &vpe_basic_control);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param vpe_mag_tuning The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_vpe_magnetometer_basic_tuning(const_buf_ref_t msg_buf, len_t msg_len,
		VpeMagnetometerBasicTuning &vpe_mag_tuning);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param vpe_accel_tuning The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_vpe_accelerometer_basic_tuning(const_buf_ref_t msg_buf, len_t msg_len,
		VpeAccelerometerBasicTuning &vpe_accel_tuning);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param filter_startup_gyro_bias The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_filter_startup_gyro_bias(const_buf_ref_t msg_buf, len_t msg_len,
		FilterStartupGyroBias &filter_startup_gyro_bias);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param mag_calibration The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_magnetometer_calibration(const_buf_ref_t msg_buf, len_t msg_len,
		MagnetometerCalibration &mag_calibration);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param accel_calibration The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_accelerometer_calibration(const_buf_ref_t msg_buf, len_t msg_len,
		AccelerometerCalibration &accel_calibration);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param ref_frame_rotation The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_reference_frame_rotation(const_buf_ref_t msg_buf, len_t msg_len,
		ReferenceFrameRotation &ref_frame_rotation);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param delta_theta_config The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_delta_theta_delta_velocity_config(const_buf_ref_t msg_buf, len_t msg_len,
		DeltaThetaDeltaVelocityConfig &delta_theta_config);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param gyro_calibration The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_gyro_calibration(const_buf_ref_t msg_buf, len_t msg_len, GyroCalibration &gyro_calibration);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param imu_filtering_config The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_imu_filtering_config(const_buf_ref_t msg_buf, len_t msg_len, ImuFilteringConfig &imu_filtering_config);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param real_time_hsi_control The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_real_time_hsi_control(const_buf_ref_t msg_buf, len_t msg_len,
				      RealTimeHsiControl &real_time_hsi_control);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param velocity_aiding_measurement The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_velocity_aiding_measurement(const_buf_ref_t msg_buf, len_t msg_len,
		VelocityAidingMeasurement &velocity_aiding_measurement);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param velocity_aiding_control The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_velocity_aiding_control(const_buf_ref_t msg_buf, len_t msg_len,
					VelocityAidingControl &velocity_aiding_control);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param ref_model_config The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_reference_model_config(const_buf_ref_t msg_buf, len_t msg_len, ReferenceModelConfig &ref_model_config);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param heave_basic_config The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_heave_basic_config(const_buf_ref_t msg_buf, len_t msg_len, HeaveBasicConfig &heave_basic_config);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param model The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_model(const_buf_ref_t msg_buf, len_t msg_len, Model &model);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param hardware_version The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_hardware_version(const_buf_ref_t msg_buf, len_t msg_len, HardwareVersion &hardware_version);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param serial_number The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_serial_number(const_buf_ref_t msg_buf, len_t msg_len, SerialNumber &serial_number);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param firmware_version The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_firmware_version(const_buf_ref_t msg_buf, len_t msg_len, FirmwareVersion &firmware_version);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param sync_status The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_sync_status(const_buf_ref_t msg_buf, len_t msg_len, SyncStatus &sync_status);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param yaw_pitch_roll The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_yaw_pitch_roll(const_buf_ref_t msg_buf, len_t msg_len, YawPitchRoll &yaw_pitch_roll);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param quaternion The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_quaternion(const_buf_ref_t msg_buf, len_t msg_len, Quaternion &quaternion);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param quaternion_compensated_imu The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_quaternion_compensated_imu(const_buf_ref_t msg_buf, len_t msg_len,
		QuaternionCompensatedImu &quaternion_compensated_imu);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param yaw_pitch_roll_compensated_imu The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_yaw_pitch_roll_compensated_imu(const_buf_ref_t msg_buf, len_t msg_len,
		YawPitchRollCompensatedImu &yaw_pitch_roll_compensated_imu);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param yaw_pitch_roll_linear_accel_gyro The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_yaw_pitch_roll_linear_accel_gyro(const_buf_ref_t msg_buf, len_t msg_len,
		YawPitchRollLinearAccelGyro &yaw_pitch_roll_linear_accel_gyro);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param yaw_pitch_roll_linear_inertial_accel_gyro The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_yaw_pitch_roll_linear_inertial_accel_gyro(const_buf_ref_t msg_buf, len_t msg_len,
		YawPitchRollLinearInertialAccelGyro &yaw_pitch_roll_linear_inertial_accel_gyro);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param compensated_magnetometer The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_compensated_magnetometer(const_buf_ref_t msg_buf, len_t msg_len,
		CompensatedMagnetometer &compensated_magnetometer);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param compensated_accelerometer The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_compensated_accelerometer(const_buf_ref_t msg_buf, len_t msg_len,
		CompensatedAccelerometer &compensated_accelerometer);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param compensated_gyro The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_compensated_gyro(const_buf_ref_t msg_buf, len_t msg_len, CompensatedGyro &compensated_gyro);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param compensated_imu The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_compensated_imu(const_buf_ref_t msg_buf, len_t msg_len, CompensatedImu &compensated_imu);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param imu_measurements The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_imu_measurements(const_buf_ref_t msg_buf, len_t msg_len, ImuMeasurements &imu_measurements);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param delta_theta_delta_velocity The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_delta_theta_delta_velocity(const_buf_ref_t msg_buf, len_t msg_len,
		DeltaThetaDeltaVelocity &delta_theta_delta_velocity);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param real_time_hsi_results The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_real_time_hsi_results(const_buf_ref_t msg_buf, len_t msg_len,
				      RealTimeHsiResults &real_time_hsi_results);
/**
 * @brief Given a complete message buffer, extract the register information into the provided struct
 *
 * @param msg_buf The message buffer.
 * @param msg_len The length of the message.
 * @param heave_and_heave_rate The register struct to populate.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode parse_heave_and_heave_rate(const_buf_ref_t msg_buf, len_t msg_len, HeaveAndHeaveRate &heave_and_heave_rate);

} // namespace msg

} // namespace vn
