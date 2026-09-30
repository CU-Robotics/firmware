/**
 * @file register_ops.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines a template for register operations on the VectorNav VN100. This maps each register to
 *        its corresponding parse and creation functions.
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
#include "msg_parsing.hpp"
#include "msg_creation.hpp"

namespace vn
{

template<typename T>
struct RegisterOps;

template<>
struct RegisterOps<WriteSettings> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, WriteSettings &write_settings)
	{
		return msg::parse_write_settings(msg_buf, msg_len, write_settings);
	}
	static ErrorCode write(const WriteSettings &write_settings, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_write_settings(msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<RestoreFactorySettings> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      RestoreFactorySettings &restore_factory_settings)
	{
		return msg::parse_restore_factory_settings(msg_buf, msg_len, restore_factory_settings);
	}
	static ErrorCode write(const RestoreFactorySettings &restore_factory_settings, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_restore_factory_settings(msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<Reset> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, Reset &reset)
	{
		return msg::parse_reset(msg_buf, msg_len, reset);
	}
	static ErrorCode write(const Reset &reset, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_reset(msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<FirmwareUpdate> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, FirmwareUpdate &firmware_update)
	{
		return ErrorCode::InvalidCommand; // not supported
	}
	static ErrorCode write(const FirmwareUpdate &firmware_update, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // not supported
	}
};

template<>
struct RegisterOps<KnownMagneticDisturbance> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, KnownMagneticDisturbance &mag_disturbance)
	{
		return msg::parse_known_magnetic_disturbance(msg_buf, msg_len, mag_disturbance);
	}
	static ErrorCode write(const KnownMagneticDisturbance &mag_disturbance, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_known_magnetic_disturbance(mag_disturbance, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<KnownAccelerationDisturbance> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, KnownAccelerationDisturbance &accel_disturbance)
	{
		return msg::parse_known_acceleration_disturbance(msg_buf, msg_len, accel_disturbance);
	}
	static ErrorCode write(const KnownAccelerationDisturbance &accel_disturbance, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_known_acceleration_disturbance(accel_disturbance, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<AsyncOutputEnable> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, AsyncOutputEnable &async_output_enable)
	{
		return msg::parse_async_output_enable(msg_buf, msg_len, async_output_enable);
	}
	static ErrorCode write(const AsyncOutputEnable &async_output_enable, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_async_output_enable(async_output_enable, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<SetGyroBias> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, SetGyroBias &set_gyro_bias)
	{
		return msg::parse_set_gyro_bias(msg_buf, msg_len, set_gyro_bias);
	}
	static ErrorCode write(const SetGyroBias &set_gyro_bias, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_set_gyro_bias(msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<PollBinaryOutputMessage> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, PollBinaryOutputMessage &poll_binary_output_msg)
	{
		return ErrorCode::InvalidCommand; // not supported
	}
	static ErrorCode write(const PollBinaryOutputMessage &poll_binary_output_msg, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // not supported
	}
};

template<>
struct RegisterOps<UserTag> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, UserTag &user_tag)
	{
		return msg::parse_user_tag(msg_buf, msg_len, user_tag);
	}
	static ErrorCode write(const UserTag &user_tag, msg::buf_ref_t msg_buf,  msg::len_t &msg_len)
	{
		return msg::create_user_tag(user_tag, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<Baudrate> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, Baudrate &baud_rate_reg)
	{
		return msg::parse_baudrate(msg_buf, msg_len, baud_rate_reg);
	}
	static ErrorCode write(const Baudrate &baud_rate_reg, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_baudrate(baud_rate_reg, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<AsyncDataOutputType> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, AsyncDataOutputType &async_data_output_type)
	{
		return msg::parse_async_data_output_type(msg_buf, msg_len, async_data_output_type);
	}
	static ErrorCode write(const AsyncDataOutputType &async_data_output_type, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_async_data_output_type(async_data_output_type, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<AsyncDataOutputFreq> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, AsyncDataOutputFreq &async_data_output_freq)
	{
		return msg::parse_async_data_output_freq(msg_buf, msg_len, async_data_output_freq);
	}
	static ErrorCode write(const AsyncDataOutputFreq &async_data_output_freq, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_async_data_output_freq(async_data_output_freq, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<CommunicationProtocolControl> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      CommunicationProtocolControl &comm_protocol_control)
	{
		return msg::parse_communication_protocol_control(msg_buf, msg_len, comm_protocol_control);
	}
	static ErrorCode write(const CommunicationProtocolControl &comm_protocol_control, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_communication_protocol_control(comm_protocol_control, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<SyncControl> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, SyncControl &sync_control)
	{
		return msg::parse_sync_control(msg_buf, msg_len, sync_control);
	}
	static ErrorCode write(const SyncControl &sync_control, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_sync_control(sync_control, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<LegacyCompatibilitySettings> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, LegacyCompatibilitySettings &legacy_settings)
	{
		return msg::parse_legacy_compatibility_settings(msg_buf, msg_len, legacy_settings);
	}
	static ErrorCode write(const LegacyCompatibilitySettings &legacy_settings, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_legacy_compatibility_settings(legacy_settings, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<BinaryOutputMessageConfig1> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      BinaryOutputMessageConfig1 &binary_output_msg_config_1)
	{
		return msg::parse_binary_output_message_config_1(msg_buf, msg_len, binary_output_msg_config_1);
	}
	static ErrorCode write(const BinaryOutputMessageConfig1 &binary_output_msg_config_1, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_binary_output_message_config_1(binary_output_msg_config_1, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<BinaryOutputMessageConfig2> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      BinaryOutputMessageConfig2 &binary_output_msg_config_2)
	{
		return msg::parse_binary_output_message_config_2(msg_buf, msg_len, binary_output_msg_config_2);
	}
	static ErrorCode write(const BinaryOutputMessageConfig2 &binary_output_msg_config_2, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_binary_output_message_config_2(binary_output_msg_config_2, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<BinaryOutputMessageConfig3> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      BinaryOutputMessageConfig3 &binary_output_msg_config_3)
	{
		return msg::parse_binary_output_message_config_3(msg_buf, msg_len, binary_output_msg_config_3);
	}
	static ErrorCode write(const BinaryOutputMessageConfig3 &binary_output_msg_config_3, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_binary_output_message_config_3(binary_output_msg_config_3, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<MagneticGravityReference> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, MagneticGravityReference &mag_grav_ref)
	{
		return msg::parse_magnetic_gravity_reference(msg_buf, msg_len, mag_grav_ref);
	}
	static ErrorCode write(const MagneticGravityReference &mag_grav_ref, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_magnetic_gravity_reference(mag_grav_ref, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<VpeBasicControl> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, VpeBasicControl &vpe_basic_control)
	{
		return msg::parse_vpe_basic_control(msg_buf, msg_len, vpe_basic_control);
	}
	static ErrorCode write(const VpeBasicControl &vpe_basic_control, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_vpe_basic_control(vpe_basic_control, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<VpeMagnetometerBasicTuning> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, VpeMagnetometerBasicTuning &vpe_mag_tuning)
	{
		return msg::parse_vpe_magnetometer_basic_tuning(msg_buf, msg_len, vpe_mag_tuning);
	}
	static ErrorCode write(const VpeMagnetometerBasicTuning &vpe_mag_tuning, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_vpe_magnetometer_basic_tuning(vpe_mag_tuning, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<VpeAccelerometerBasicTuning> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, VpeAccelerometerBasicTuning &vpe_accel_tuning)
	{
		return msg::parse_vpe_accelerometer_basic_tuning(msg_buf, msg_len, vpe_accel_tuning);
	}
	static ErrorCode write(const VpeAccelerometerBasicTuning &vpe_accel_tuning, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_vpe_accelerometer_basic_tuning(vpe_accel_tuning, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<FilterStartupGyroBias> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, FilterStartupGyroBias &filter_startup_gyro_bias)
	{
		return msg::parse_filter_startup_gyro_bias(msg_buf, msg_len, filter_startup_gyro_bias);
	}
	static ErrorCode write(const FilterStartupGyroBias &filter_startup_gyro_bias, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_filter_startup_gyro_bias(filter_startup_gyro_bias, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<MagnetometerCalibration> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, MagnetometerCalibration &mag_calibration)
	{
		return msg::parse_magnetometer_calibration(msg_buf, msg_len, mag_calibration);
	}
	static ErrorCode write(const MagnetometerCalibration &mag_calibration, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_magnetometer_calibration(mag_calibration, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<AccelerometerCalibration> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, AccelerometerCalibration &accel_calibration)
	{
		return msg::parse_accelerometer_calibration(msg_buf, msg_len, accel_calibration);
	}
	static ErrorCode write(const AccelerometerCalibration &accel_calibration, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_accelerometer_calibration(accel_calibration, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<ReferenceFrameRotation> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, ReferenceFrameRotation &ref_frame_rotation)
	{
		return msg::parse_reference_frame_rotation(msg_buf, msg_len, ref_frame_rotation);
	}
	static ErrorCode write(const ReferenceFrameRotation &ref_frame_rotation, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_reference_frame_rotation(ref_frame_rotation, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<DeltaThetaDeltaVelocityConfig> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      DeltaThetaDeltaVelocityConfig &delta_theta_config)
	{
		return msg::parse_delta_theta_delta_velocity_config(msg_buf, msg_len, delta_theta_config);
	}
	static ErrorCode write(const DeltaThetaDeltaVelocityConfig &delta_theta_config, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_delta_theta_delta_velocity_config(delta_theta_config, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<GyroCalibration> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, GyroCalibration &gyro_calibration)
	{
		return msg::parse_gyro_calibration(msg_buf, msg_len, gyro_calibration);
	}
	static ErrorCode write(const GyroCalibration &gyro_calibration, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_gyro_calibration(gyro_calibration, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<ImuFilteringConfig> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, ImuFilteringConfig &imu_filtering_config)
	{
		return msg::parse_imu_filtering_config(msg_buf, msg_len, imu_filtering_config);
	}
	static ErrorCode write(const ImuFilteringConfig &imu_filtering_config, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_imu_filtering_config(imu_filtering_config, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<RealTimeHsiControl> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, RealTimeHsiControl &real_time_hsi_control)
	{
		return msg::parse_real_time_hsi_control(msg_buf, msg_len, real_time_hsi_control);
	}
	static ErrorCode write(const RealTimeHsiControl &real_time_hsi_control, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_real_time_hsi_control(real_time_hsi_control, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<VelocityAidingMeasurement> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      VelocityAidingMeasurement &velocity_aiding_measurement)
	{
		return msg::parse_velocity_aiding_measurement(msg_buf, msg_len, velocity_aiding_measurement);
	}
	static ErrorCode write(const VelocityAidingMeasurement &velocity_aiding_measurement, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_velocity_aiding_measurement(velocity_aiding_measurement, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<VelocityAidingControl> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, VelocityAidingControl &velocity_aiding_control)
	{
		return msg::parse_velocity_aiding_control(msg_buf, msg_len, velocity_aiding_control);
	}
	static ErrorCode write(const VelocityAidingControl &velocity_aiding_control, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return msg::create_velocity_aiding_control(velocity_aiding_control, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<ReferenceModelConfig> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, ReferenceModelConfig &ref_model_config)
	{
		return msg::parse_reference_model_config(msg_buf, msg_len, ref_model_config);
	}
	static ErrorCode write(const ReferenceModelConfig &ref_model_config, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_reference_model_config(ref_model_config, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<HeaveBasicConfig> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, HeaveBasicConfig &heave_basic_config)
	{
		return msg::parse_heave_basic_config(msg_buf, msg_len, heave_basic_config);
	}
	static ErrorCode write(const HeaveBasicConfig &heave_basic_config, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return msg::create_heave_basic_config(heave_basic_config, msg_buf, msg_len);
	}
};

template<>
struct RegisterOps<Model> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, Model &model)
	{
		return msg::parse_model(msg_buf, msg_len, model);
	}
	static ErrorCode write(const Model &model, msg::buf_ref_t msg_buf,  msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<HardwareVersion> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, HardwareVersion &hardware_version)
	{
		return msg::parse_hardware_version(msg_buf, msg_len, hardware_version);
	}
	static ErrorCode write(const HardwareVersion &hardware_version, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<SerialNumber> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, SerialNumber &serial_number)
	{
		return msg::parse_serial_number(msg_buf, msg_len, serial_number);
	}
	static ErrorCode write(const SerialNumber &serial_number, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<FirmwareVersion> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, FirmwareVersion &firmware_version)
	{
		return msg::parse_firmware_version(msg_buf, msg_len, firmware_version);
	}
	static ErrorCode write(const FirmwareVersion &firmware_version, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<SyncStatus> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, SyncStatus &sync_status)
	{
		return msg::parse_sync_status(msg_buf, msg_len, sync_status);
	}
	static ErrorCode write(const SyncStatus &sync_status, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<YawPitchRoll> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, YawPitchRoll &yaw_pitch_roll)
	{
		return msg::parse_yaw_pitch_roll(msg_buf, msg_len, yaw_pitch_roll);
	}
	static ErrorCode write(const YawPitchRoll &yaw_pitch_roll, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<Quaternion> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, Quaternion &quaternion)
	{
		return msg::parse_quaternion(msg_buf, msg_len, quaternion);
	}
	static ErrorCode write(const Quaternion &quaternion, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<QuaternionCompensatedImu> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      QuaternionCompensatedImu &quaternion_compensated_imu)
	{
		return msg::parse_quaternion_compensated_imu(msg_buf, msg_len, quaternion_compensated_imu);
	}
	static ErrorCode write(const QuaternionCompensatedImu &quaternion_compensated_imu, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<YawPitchRollCompensatedImu> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      YawPitchRollCompensatedImu &yaw_pitch_roll_compensated_imu)
	{
		return msg::parse_yaw_pitch_roll_compensated_imu(msg_buf, msg_len, yaw_pitch_roll_compensated_imu);
	}
	static ErrorCode write(const YawPitchRollCompensatedImu &yaw_pitch_roll_compensated_imu, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<YawPitchRollLinearAccelGyro> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      YawPitchRollLinearAccelGyro &yaw_pitch_roll_linear_accel_gyro)
	{
		return msg::parse_yaw_pitch_roll_linear_accel_gyro(msg_buf, msg_len, yaw_pitch_roll_linear_accel_gyro);
	}
	static ErrorCode write(const YawPitchRollLinearAccelGyro &yaw_pitch_roll_linear_accel_gyro, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<YawPitchRollLinearInertialAccelGyro> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      YawPitchRollLinearInertialAccelGyro &yaw_pitch_roll_linear_inertial_accel_gyro)
	{
		return msg::parse_yaw_pitch_roll_linear_inertial_accel_gyro(msg_buf, msg_len,
				yaw_pitch_roll_linear_inertial_accel_gyro);
	}
	static ErrorCode write(const YawPitchRollLinearInertialAccelGyro &yaw_pitch_roll_linear_inertial_accel_gyro,
			       msg::buf_ref_t msg_buf,  msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<CompensatedMagnetometer> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      CompensatedMagnetometer &compensated_magnetometer)
	{
		return msg::parse_compensated_magnetometer(msg_buf, msg_len, compensated_magnetometer);
	}
	static ErrorCode write(const CompensatedMagnetometer &compensated_magnetometer, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<CompensatedAccelerometer> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      CompensatedAccelerometer &compensated_accelerometer)
	{
		return msg::parse_compensated_accelerometer(msg_buf, msg_len, compensated_accelerometer);
	}
	static ErrorCode write(const CompensatedAccelerometer &compensated_accelerometer, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<CompensatedGyro> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, CompensatedGyro &compensated_gyro)
	{
		return msg::parse_compensated_gyro(msg_buf, msg_len, compensated_gyro);
	}
	static ErrorCode write(const CompensatedGyro &compensated_gyro, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<CompensatedImu> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, CompensatedImu &compensated_imu)
	{
		return msg::parse_compensated_imu(msg_buf, msg_len, compensated_imu);
	}
	static ErrorCode write(const CompensatedImu &compensated_imu, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<ImuMeasurements> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, ImuMeasurements &imu_measurements)
	{
		return msg::parse_imu_measurements(msg_buf, msg_len, imu_measurements);
	}
	static ErrorCode write(const ImuMeasurements &imu_measurements, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<DeltaThetaDeltaVelocity> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len,
			      DeltaThetaDeltaVelocity &delta_theta_delta_velocity)
	{
		return msg::parse_delta_theta_delta_velocity(msg_buf, msg_len, delta_theta_delta_velocity);
	}
	static ErrorCode write(const DeltaThetaDeltaVelocity &delta_theta_delta_velocity, msg::buf_ref_t msg_buf,
			       msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<RealTimeHsiResults> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, RealTimeHsiResults &real_time_hsi_results)
	{
		return msg::parse_real_time_hsi_results(msg_buf, msg_len, real_time_hsi_results);
	}
	static ErrorCode write(const RealTimeHsiResults &real_time_hsi_results, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

template<>
struct RegisterOps<HeaveAndHeaveRate> {
	static ErrorCode read(msg::const_buf_ref_t msg_buf, msg::len_t msg_len, HeaveAndHeaveRate &heave_and_heave_rate)
	{
		return msg::parse_heave_and_heave_rate(msg_buf, msg_len, heave_and_heave_rate);
	}
	static ErrorCode write(const HeaveAndHeaveRate &heave_and_heave_rate, msg::buf_ref_t msg_buf, msg::len_t &msg_len)
	{
		return ErrorCode::InvalidCommand; // read only
	}
};

} // namespace vn
