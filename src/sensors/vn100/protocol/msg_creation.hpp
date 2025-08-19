/**
 * @file msg_creation.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines the message creation functions for the VectorNav VN100. This includes functions to create messages, calculate checksums,
 *	      and handle message formatting.
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
#include "registers.hpp"

namespace vn
{

namespace msg
{

/// @see msg_creation.cpp for static inline message creation helpers

/**
 * @brief Create a complete VNRRG message with the given register ID
 *
 * @param reg_id Register ID to create the message for
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_read_register(const uint8_t reg_id, buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNWNV message to write settings
 *
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_write_settings(buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNRST message to restore factory settings
 *
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_restore_factory_settings(buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNRST message to reset the device
 *
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_reset(buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNFUP message to perform a firmware update
 *
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_firmware_update(buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNKMD message to set a known magnetic disturbance
 *
 * @param mag_disturbance A fully populated KnownMagneticDisturbance struct containing the disturbance data
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_known_magnetic_disturbance(const KnownMagneticDisturbance &mag_disturbance, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Create a complete VNKAD message to set a known acceleration disturbance
 *
 * @param accel_disturbance A fully populated KnownAccelerationDisturbance struct containing the disturbance data
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_known_acceleration_disturbance(const KnownAccelerationDisturbance &accel_disturbance,
		buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNASY message to enable or disable async output
 *
 * @param async_output_enable A fully populated AsyncOutputEnable struct containing the enable state
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_async_output_enable(const AsyncOutputEnable &async_output_enable, buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNSGB message to set gyro bias
 *
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_set_gyro_bias(buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Create a complete VNBOM message to poll binary output messages
 *
 * @param msg_buf Buffer to write the message to
 * @param msg_len Length of the message after creation
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
 */
ErrorCode create_poll_binary_output_message(buf_ref_t msg_buf, len_t &msg_len);

// Configuration Registers
// Create functions make a fully assembled VNWRG message with the given register data

/**
 * @brief Creates a VNWRG message for the UserTag register using the provided struct data.
 *
 * @param user_tag A fully populated UserTag struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_user_tag(const UserTag &user_tag, buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the Baudrate register using the provided struct data.
 *
 * @param baudrate A fully populated Baudrate struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_baudrate(const Baudrate &baudrate, buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the AsyncDataOutputType register using the provided struct data.
 *
 * @param async_data_output_type A fully populated AsyncDataOutputType struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_async_data_output_type(const AsyncDataOutputType &async_data_output_type, buf_ref_t msg_buf,
					len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the AsyncDataOutputFreq register using the provided struct data.
 *
 * @param async_data_output_freq A fully populated AsyncDataOutputFreq struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_async_data_output_freq(const AsyncDataOutputFreq &async_data_output_freq, buf_ref_t msg_buf,
					len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the CommunicationProtocolControl register using the provided struct data.
 *
 * @param comm_protocol_control A fully populated CommunicationProtocolControl struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_communication_protocol_control(const CommunicationProtocolControl &comm_protocol_control,
		buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the SyncControl register using the provided struct data.
 *
 * @param sync_control A fully populated SyncControl struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_sync_control(const SyncControl &sync_control, buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the LegacyCompatibilitySettings register using the provided struct data.
 *
 * @param legacy_settings A fully populated LegacyCompatibilitySettings struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_legacy_compatibility_settings(const LegacyCompatibilitySettings &legacy_settings, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the BinaryOutputMessageConfig1 register using the provided struct data.
 *
 * @param binary_output_msg_config_1 A fully populated BinaryOutputMessageConfig1 struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_binary_output_message_config_1(const BinaryOutputMessageConfig1 &binary_output_msg_config_1,
		buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the BinaryOutputMessageConfig2 register using the provided struct data.
 *
 * @param binary_output_msg_config_2 A fully populated BinaryOutputMessageConfig2 struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_binary_output_message_config_2(const BinaryOutputMessageConfig2 &binary_output_msg_config_2,
		buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the BinaryOutputMessageConfig3 register using the provided struct data.
 *
 * @param binary_output_msg_config_3 A fully populated BinaryOutputMessageConfig3 struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_binary_output_message_config_3(const BinaryOutputMessageConfig3 &binary_output_msg_config_3,
		buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the MagneticGravityReference register using the provided struct data.
 *
 * @param mag_grav_ref A fully populated MagneticGravityReference struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_magnetic_gravity_reference(const MagneticGravityReference &mag_grav_ref, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the VpeBasicControl register using the provided struct data.
 *
 * @param vpe_basic_control A fully populated VpeBasicControl struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_vpe_basic_control(const VpeBasicControl &vpe_basic_control, buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the VpeMagnetometerBasicTuning register using the provided struct data.
 *
 * @param vpe_mag_tuning A fully populated VpeMagnetometerBasicTuning struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_vpe_magnetometer_basic_tuning(const VpeMagnetometerBasicTuning &vpe_mag_tuning, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the VpeAccelerometerBasicTuning register using the provided struct data.
 *
 * @param vpe_accel_tuning A fully populated VpeAccelerometerBasicTuning struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_vpe_accelerometer_basic_tuning(const VpeAccelerometerBasicTuning &vpe_accel_tuning, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the FilterStartupGyroBias register using the provided struct data.
 *
 * @param filter_startup_gyro_bias A fully populated FilterStartupGyroBias struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_filter_startup_gyro_bias(const FilterStartupGyroBias &filter_startup_gyro_bias, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the MagnetometerCalibration register using the provided struct data.
 *
 * @param mag_calibration A fully populated MagnetometerCalibration struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_magnetometer_calibration(const MagnetometerCalibration &mag_calibration, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the AccelerometerCalibration register using the provided struct data.
 *
 * @param accel_calibration A fully populated AccelerometerCalibration struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_accelerometer_calibration(const AccelerometerCalibration &accel_calibration, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the ReferenceFrameRotation register using the provided struct data.
 *
 * @param ref_frame_rotation A fully populated ReferenceFrameRotation struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_reference_frame_rotation(const ReferenceFrameRotation &ref_frame_rotation, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the DeltaThetaDeltaVelocityConfig register using the provided struct data.
 *
 * @param delta_theta_config A fully populated DeltaThetaDeltaVelocityConfig struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_delta_theta_delta_velocity_config(const DeltaThetaDeltaVelocityConfig &delta_theta_config,
		buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the GyroCalibration register using the provided struct data.
 *
 * @param gyro_calibration A fully populated GyroCalibration struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_gyro_calibration(const GyroCalibration &gyro_calibration, buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the ImuFilteringConfig register using the provided struct data.
 *
 * @param imu_filtering_config A fully populated ImuFilteringConfig struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_imu_filtering_config(const ImuFilteringConfig &imu_filtering_config, buf_ref_t msg_buf,
				      len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the RealTimeHsiControl register using the provided struct data.
 *
 * @param real_time_hsi_control A fully populated RealTimeHsiControl struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_real_time_hsi_control(const RealTimeHsiControl &real_time_hsi_control, buf_ref_t msg_buf,
				       len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the VelocityAidingMeasurement register using the provided struct data.
 *
 * @param velocity_aiding_measurements A fully populated VelocityAidingMeasurement struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_velocity_aiding_measurement(const VelocityAidingMeasurement &velocity_aiding_measurements,
		buf_ref_t msg_buf, len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the VelocityAidingControl register using the provided struct data.
 *
 * @param velocity_aiding_control A fully populated VelocityAidingControl struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_velocity_aiding_control(const VelocityAidingControl &velocity_aiding_control, buf_ref_t msg_buf,
		len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the ReferenceModelConfig register using the provided struct data.
 *
 * @param ref_model_config A fully populated ReferenceModelConfig struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_reference_model_config(const ReferenceModelConfig &ref_model_config, buf_ref_t msg_buf,
					len_t &msg_len);
/**
 * @brief Creates a VNWRG message for the HeaveBasicConfig register using the provided struct data.
 *
 * @param heave_basic_config A fully populated HeaveBasicConfig struct containing the tag to be set.
 * @param msg_buf Buffer to write the message to.
 * @param msg_len Length of the message after creation.
 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure.
 */
ErrorCode create_heave_basic_config(const HeaveBasicConfig &heave_basic_config, buf_ref_t msg_buf, len_t &msg_len);


// Measurement Registers (readonly)
// They dont get a create function, as they are read only.

} // namespace msg

} // namespace vn
