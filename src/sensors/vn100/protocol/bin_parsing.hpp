/**
 * @file bin_parsing.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines the binary parsing functions for the VectorNav VN100. This includes functions to parse binary messages, validate checksums,
 *	      and handle binary message formatting. This also manages the conversion from raw binary message bytes to structs.
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

namespace vn
{

namespace bin
{

/**
 * @brief Extracts and validates the group byte from the binary message buffer.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode extract_group(const_bin_buf_ref_t buf, BinaryMessage &msg);
/**
 * @brief Extracts and validates the type bytes from the binary message buffer.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode extract_group_types(const_bin_buf_ref_t buf, BinaryMessage &msg);
/**
 * @brief Extracts and validates the crc from the binary message buffer.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode verify_crc(const_bin_buf_ref_t buf, BinaryMessage &msg);
/**
 * @brief Does a full extraction and validation of all aspects of an incoming message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode verify_message(const_bin_buf_ref_t buf, BinaryMessage &msg);
/**
 * @brief Determines the offset from buffer start for a specific message group+type.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param offset The offset to populate.
 * @param group_bitfield The group bitfield to match.
 * @param type_bitfield The type bitfield to match.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode get_offset_by_type(const_bin_buf_ref_t buf, BinaryMessage &msg, msg::len_t &offset, uint8_t group_bitfield,
			     uint16_t type_bitfield);
/**
 * @brief Given one message, determine if another is the same configuration
 *
 * @param msg1 The first message to compare.
 * @param msg2 The second message to compare.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode compare_messages(BinaryMessage &msg1, BinaryMessage &msg2);

/**
 * @brief Parse the common time startup message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param time_startup The CommonTimeStartup struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_time_startup(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonTimeStartup &time_startup);
/**
 * @brief Parse the Common Type Sync-In message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param sync_in The CommonTypeSyncIn struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_type_sync_in(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonTypeSyncIn &sync_in);
/**
 * @brief Parse the Common Yaw-Pitch-Roll (YPR) message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param ypr The CommonYpr struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_ypr(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonYpr &ypr);
/**
 * @brief Parse the Common Quaternion orientation message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param quaternion The CommonQuaternion struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_quaternion(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonQuaternion &quaternion);
/**
 * @brief Parse the Common Angular Rate message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param angular_rate The CommonAngularRate struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_angular_rate(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonAngularRate &angular_rate);
/**
 * @brief Parse the Common Acceleration message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param accel The CommonAccel struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_accel(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonAccel &accel);
/**
 * @brief Parse the Common IMU (accel/gyro/temp/etc.) message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param imu The CommonImu struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_imu(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonImu &imu);
/**
 * @brief Parse the Common Magnetometer/Pressure message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param mag_pres The CommonMagPres struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_mag_pres(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonMagPres &mag_pres);
/**
 * @brief Parse the Common integrated deltas (e.g., delta-theta/velocity) message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param deltas The CommonDeltas struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_deltas(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonDeltas &deltas);
/**
 * @brief Parse the Common Sync-In count message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param sync_in_cnt The CommonSyncInCnt struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_common_sync_in_cnt(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonSyncInCnt &sync_in_cnt);

/**
 * @brief Parse the Time Startup message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param time_startup The TimeStartup struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_time_startup(const_bin_buf_ref_t buf, BinaryMessage &msg, TimeStartup &time_startup);
/**
 * @brief Parse the Time Sync-In message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param time_sync_in The TimeSyncIn struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_time_sync_in(const_bin_buf_ref_t buf, BinaryMessage &msg, TimeSyncIn &time_sync_in);
/**
 * @brief Parse the Sync-In counter message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param sync_in_cnt The SyncInCnt struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_sync_in_cnt(const_bin_buf_ref_t buf, BinaryMessage &msg, SyncInCnt &sync_in_cnt);
/**
 * @brief Parse the Sync-Out counter message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param sync_out_cnt The SyncOutCnt struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_sync_out_cnt(const_bin_buf_ref_t buf, BinaryMessage &msg, SyncOutCnt &sync_out_cnt);

/**
 * @brief Parse the IMU status/health message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param imu_status The ImuStatus struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_imu_status(const_bin_buf_ref_t buf, BinaryMessage &msg, ImuStatus &imu_status);
/**
 * @brief Parse the uncompensated magnetometer reading message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param uncomp_mag The UncompMag struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_uncomp_mag(const_bin_buf_ref_t buf, BinaryMessage &msg, UncompMag &uncomp_mag);
/**
 * @brief Parse the uncompensated accelerometer reading message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param uncomp_acc The UncompAcc struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_uncomp_acc(const_bin_buf_ref_t buf, BinaryMessage &msg, UncompAcc &uncomp_acc);
/**
 * @brief Parse the uncompensated gyroscope reading message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param uncomp_gyro The UncompGyro struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_uncomp_gyro(const_bin_buf_ref_t buf, BinaryMessage &msg, UncompGyro &uncomp_gyro);
/**
 * @brief Parse the temperature message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param temperature The Temperature struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_temperature(const_bin_buf_ref_t buf, BinaryMessage &msg, Temperature &temperature);
/**
 * @brief Parse the pressure message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param pressure The Pressure struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_pressure(const_bin_buf_ref_t buf, BinaryMessage &msg, Pressure &pressure);
/**
 * @brief Parse the integrated delta-theta (angle increment) message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param delta_theta The DeltaTheta struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_delta_theta(const_bin_buf_ref_t buf, BinaryMessage &msg, DeltaTheta &delta_theta);
/**
 * @brief Parse the integrated delta-velocity message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param delta_vel The DeltaVel struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_delta_vel(const_bin_buf_ref_t buf, BinaryMessage &msg, DeltaVel &delta_vel);
/**
 * @brief Parse the calibrated magnetic field message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param mag The Mag struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_mag(const_bin_buf_ref_t buf, BinaryMessage &msg, Mag &mag);
/**
 * @brief Parse the calibrated acceleration message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param accel The Accel struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_accel(const_bin_buf_ref_t buf, BinaryMessage &msg, Accel &accel);
/**
 * @brief Parse the calibrated angular rate message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param angular_rate The AngularRate struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_angular_rate(const_bin_buf_ref_t buf, BinaryMessage &msg, AngularRate &angular_rate);
/**
 * @brief Parse the sensor saturation/overrange flags message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param sens_sat The SensSat struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_sens_sat(const_bin_buf_ref_t buf, BinaryMessage &msg, SensSat &sens_sat);

/**
 * @brief Parse the AHRS status message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param ahrs_status The AhrsStatus struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_ahrs_status(const_bin_buf_ref_t buf, BinaryMessage &msg, AhrsStatus &ahrs_status);
/**
 * @brief Parse the Yaw-Pitch-Roll (YPR) attitude message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param ypr The Ypr struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_ypr(const_bin_buf_ref_t buf, BinaryMessage &msg, Ypr &ypr);
/**
 * @brief Parse the Quaternion attitude message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param quaternion The Quaternion struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_quaternion(const_bin_buf_ref_t buf, BinaryMessage &msg, Quaternion &quaternion);
/**
 * @brief Parse the Direction Cosine Matrix (DCM) attitude message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param dcm The Dcm struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_dcm(const_bin_buf_ref_t buf, BinaryMessage &msg, Dcm &dcm);
/**
 * @brief Parse the magnetic field in NED frame message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param mag_ned The MagNed struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_mag_ned(const_bin_buf_ref_t buf, BinaryMessage &msg, MagNed &mag_ned);
/**
 * @brief Parse the acceleration in NED frame message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param accel_ned The AccelNed struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_accel_ned(const_bin_buf_ref_t buf, BinaryMessage &msg, AccelNed &accel_ned);
/**
 * @brief Parse the linear body-frame acceleration (gravity removed) message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param lin_body_acc The LinBodyAcc struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_lin_body_acc(const_bin_buf_ref_t buf, BinaryMessage &msg, LinBodyAcc &lin_body_acc);
/**
 * @brief Parse the linear acceleration in NED frame (gravity removed) message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param lin_accel_ned The LinAccelNed struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_lin_accel_ned(const_bin_buf_ref_t buf, BinaryMessage &msg, LinAccelNed &lin_accel_ned);
/**
 * @brief Parse the YPR-with-uncertainty/uncalibrated YPR message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param ypr_u The YprU struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_ypr_u(const_bin_buf_ref_t buf, BinaryMessage &msg, YprU &ypr_u);
/**
 * @brief Parse the heave (vertical displacement) message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param heave The Heave struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_heave(const_bin_buf_ref_t buf, BinaryMessage &msg, Heave &heave);
/**
 * @brief Parse the attitude-with-uncertainty/uncalibrated attitude message.
 *
 * @param buf The binary message buffer.
 * @param msg The binary message support struct.
 * @param att_u The AttU struct to populate.
 * @return ErrorCode ErrorCode::OK on success, anything else on error.
 */
ErrorCode parse_att_u(const_bin_buf_ref_t buf, BinaryMessage &msg, AttU &att_u);

} // namespace bin

} // namespace vn
