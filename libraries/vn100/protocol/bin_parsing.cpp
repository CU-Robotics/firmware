/**
 * @file bin_parsing.cpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Implements the binary parsing functions for the VectorNav VN100. This includes functions to parse binary messages, validate checksums,
 *	      and handle binary message formatting. This also manages the conversion from raw binary message bytes to structs.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#include "bin_parsing.hpp"
#include <stdio.h>

namespace vn
{

namespace bin
{

// lookup table of type sizes
// indexed by [group bit position][type bit position]
static constexpr uint8_t binary_type_sizes[5][14] = {
	{8,  0,  8, 12, 16, 12,  0,  0, 12, 24, 20, 28,  2, 4}, // CommonGroup
	{8,  0,  0,  0,  8,  0,  0,  4,  4,  0,  0,  0,  0, 0}, // TimeGroup
	{2, 12, 12, 12,  4,  4, 16, 12, 12, 12, 12,  2,  0, 0}, // ImuGroup
	{0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0}, // Unused
	{2, 12, 16, 36, 12, 12, 12, 12, 12,  0,  0,  0, 12, 4}, // AttitudeGroup
};

// TODO: The logic of parsing the position of a specific group/type combination is messy and ineffecient.
//	 The logic should be improved and use more cacheing of results. The offsets wont ever change so we can leverage that

ErrorCode extract_group(const_bin_buf_ref_t buf, BinaryMessage &msg)
{
	msg.binary_group = 0;
	msg.num_groups = 0;
	msg.size = 0;

	if (buf[0] != START) {
		return ErrorCode::InvalidCommand; // invalid start byte
	}

	msg.binary_group = buf[1];

	// go through each bit and see what groups are present
	for (uint8_t i = 0; i < 8; i++) {
		if ((msg.binary_group & (1u << i)) != 0) {
			// largest group is 1 << 4
			if (i > 5) {
				return ErrorCode::InvalidCommand; // invalid group bit set
			}

			// store the group index and increment the number of groups
			msg.group_indices[msg.num_groups++] = i;
		}
	}

	return ErrorCode::OK;
}

ErrorCode extract_group_types(const_bin_buf_ref_t buf, BinaryMessage &msg)
{
	if (msg.num_groups == 0) {
		return ErrorCode::InvalidCommand; // no groups to process
	}

	for (uint8_t i = 0; i < msg.num_groups; i++) {
		// get the group type bitfield
		// NOTE: endianess is little-endian, so we need to swap the bytes
		msg.group_types[i] = (buf[3 + (i * 2)] << 8) | buf[2 + (i * 2)];

		// check if the group type is valid
		if (msg.group_types[i] == 0) {
			return ErrorCode::InvalidCommand; // invalid group type
		}

		// process the group types
		for (uint8_t j = 0; j < 16; j++) {
			if ((msg.group_types[i] & (1u << j)) != 0) {
				// add the size of this type to the message size
				if (binary_type_sizes[msg.group_indices[i]][j] == 0) {
					return ErrorCode::InvalidCommand; // invalid type and group combination
				}

				msg.size += binary_type_sizes[msg.group_indices[i]][j];
			}
		}
	}

	return ErrorCode::OK;
}

ErrorCode verify_crc(const_bin_buf_ref_t buf, BinaryMessage &msg)
{
	// calculate the CRC for the message
	// size corresponds to this:
	// 1 byte for group byte
	// 2 bytes for each group type
	// 2 bytes for the CRC itself
	// msg.size

	// we buf + 1 to skip the start byte
	uint16_t crc = calculate_crc(buf + 1, msg.size + 1 + (msg.num_groups * 2) + 2);

	if (crc != CRC_GOOD) {
		return ErrorCode::InvalidChecksum; // checksum mismatch
	}

	return ErrorCode::OK; // message is valid
}

ErrorCode verify_message(const_bin_buf_ref_t buf, BinaryMessage &msg)
{
	ErrorCode err;

	// extract the group information
	if ((err = extract_group(buf, msg)) != ErrorCode::OK) {
		return err; // error extracting group
	}

	// extract the group types
	if ((err = extract_group_types(buf, msg)) != ErrorCode::OK) {
		return err; // error extracting group types
	}

	// verify the CRC
	if ((err = verify_crc(buf, msg)) != ErrorCode::OK) {
		return err; // error verifying CRC
	}

	return ErrorCode::OK; // message is valid
}

ErrorCode get_offset_by_type(const_bin_buf_ref_t buf, BinaryMessage &msg, msg::len_t &offset, uint8_t group_bitfield,
			     uint16_t type_bitfield)
{
	// goal is to find the number of bytes from the start of buf that this specific group/type combination occupies

	if (!(msg.binary_group & group_bitfield)) {
		return ErrorCode::InvalidParameter; // group not present in the message
	}

	// figure out the logical index of the group
	uint8_t group_index = 0;

	for (uint8_t i = 0; i < 8; i++) {
		if (msg.binary_group & (1u << i) && group_bitfield & (1u << i)) {
			group_index = i;
			break;
		}
	}

	// offset starts at the first byte past the header
	offset = 2 + (msg.num_groups * 2);

	// go through the group types and find the offset for the specified type
	for (uint8_t i = 0; i < msg.num_groups; i++) {
		// for each bit in group types
		for (uint8_t j = 0; j < 16; j++) {
			if ((msg.group_types[i] & (1u << j)) != 0) {
				if (binary_type_sizes[msg.group_indices[i]][j] == 0) {
					return ErrorCode::InvalidParameter; // invalid type and group combination
				}

				// if this is type is set within the group types, and this is the correct group, and the type set is what were looking for
				if (msg.group_types[i] & type_bitfield && msg.group_indices[i] == group_index && 1u << j == type_bitfield) {
					// return now. we dont increment this size since we want the start of this specific type
					return ErrorCode::OK;
				}

				// add the size of this type to the offset
				offset += binary_type_sizes[msg.group_indices[i]][j];
			}
		}
	}

	return ErrorCode::InvalidParameter;
}

ErrorCode compare_messages(BinaryMessage &msg1, BinaryMessage &msg2)
{
	// Compare the group bytes
	if (msg1.binary_group != msg2.binary_group) {
		return ErrorCode::InvalidParameter; // Group byte mismatch
	}

	// Compare the group types
	for (uint8_t i = 0; i < msg1.num_groups; i++) {
		if (msg1.group_types[i] != msg2.group_types[i]) {
			return ErrorCode::InvalidParameter; // Group type mismatch
		}
	}

	return ErrorCode::OK;
}

ErrorCode parse_common_time_startup(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonTimeStartup &time_startup)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonTimeStartup::GROUP, CommonTimeStartup::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // TimeStartup type not found
	}

	memcpy(&time_startup.time_startup, buf + offset + 0, sizeof(time_startup.time_startup));

	return ErrorCode::OK;
}

ErrorCode parse_common_type_sync_in(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonTypeSyncIn &sync_in)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonTypeSyncIn::GROUP, CommonTypeSyncIn::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // SyncIn type not found
	}

	memcpy(&sync_in.time_sync_in, buf + offset + 0, sizeof(sync_in.time_sync_in));

	return ErrorCode::OK;
}

ErrorCode parse_common_ypr(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonYpr &ypr)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonYpr::GROUP, CommonYpr::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // YPR type not found
	}

	memcpy(&ypr.yaw, buf + offset + 0, sizeof(ypr.yaw));
	memcpy(&ypr.pitch, buf + offset + 4, sizeof(ypr.pitch));
	memcpy(&ypr.roll, buf + offset + 8, sizeof(ypr.roll));

	return ErrorCode::OK;
}

ErrorCode parse_common_quaternion(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonQuaternion &quaternion)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonQuaternion::GROUP, CommonQuaternion::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Quaternion type not found
	}

	memcpy(&quaternion.quaternion[0], buf + offset + 0, sizeof(quaternion.quaternion[0]));
	memcpy(&quaternion.quaternion[1], buf + offset + 4, sizeof(quaternion.quaternion[1]));
	memcpy(&quaternion.quaternion[2], buf + offset + 8, sizeof(quaternion.quaternion[2]));
	memcpy(&quaternion.quaternion[3], buf + offset + 12, sizeof(quaternion.quaternion[3]));

	return ErrorCode::OK;
}

ErrorCode parse_common_angular_rate(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonAngularRate &angular_rate)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonAngularRate::GROUP, CommonAngularRate::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // AngularRate type not found
	}

	memcpy(&angular_rate.angular_rate[0], buf + offset + 0, sizeof(angular_rate.angular_rate[0]));
	memcpy(&angular_rate.angular_rate[1], buf + offset + 4, sizeof(angular_rate.angular_rate[1]));
	memcpy(&angular_rate.angular_rate[2], buf + offset + 8, sizeof(angular_rate.angular_rate[2]));

	return ErrorCode::OK;
}

ErrorCode parse_common_accel(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonAccel &accel)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonAccel::GROUP, CommonAccel::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Accel type not found
	}

	memcpy(&accel.acc[0], buf + offset + 0, sizeof(accel.acc[0]));
	memcpy(&accel.acc[1], buf + offset + 4, sizeof(accel.acc[1]));
	memcpy(&accel.acc[2], buf + offset + 8, sizeof(accel.acc[2]));

	return ErrorCode::OK;
}

ErrorCode parse_common_imu(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonImu &imu)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonImu::GROUP, CommonImu::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // IMU type not found
	}

	memcpy(&imu.uncomp_acc[0], buf + offset + 0, sizeof(imu.uncomp_acc[0]));
	memcpy(&imu.uncomp_acc[1], buf + offset + 4, sizeof(imu.uncomp_acc[1]));
	memcpy(&imu.uncomp_acc[2], buf + offset + 8, sizeof(imu.uncomp_acc[2]));
	memcpy(&imu.uncomp_gyro[0], buf + offset + 12, sizeof(imu.uncomp_gyro[0]));
	memcpy(&imu.uncomp_gyro[1], buf + offset + 16, sizeof(imu.uncomp_gyro[1]));
	memcpy(&imu.uncomp_gyro[2], buf + offset + 20, sizeof(imu.uncomp_gyro[2]));

	return ErrorCode::OK;
}

ErrorCode parse_common_mag_pres(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonMagPres &mag_pres)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonMagPres::GROUP, CommonMagPres::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // MagPres type not found
	}

	memcpy(&mag_pres.mag[0], buf + offset + 0, sizeof(mag_pres.mag[0]));
	memcpy(&mag_pres.mag[1], buf + offset + 4, sizeof(mag_pres.mag[1]));
	memcpy(&mag_pres.mag[2], buf + offset + 8, sizeof(mag_pres.mag[2]));
	memcpy(&mag_pres.temperature, buf + offset + 12, sizeof(mag_pres.temperature));
	memcpy(&mag_pres.pressure, buf + offset + 16, sizeof(mag_pres.pressure));

	return ErrorCode::OK;
}

ErrorCode parse_common_deltas(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonDeltas &deltas)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonDeltas::GROUP, CommonDeltas::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Deltas type not found
	}

	memcpy(&deltas.delta_time, buf + offset + 0, sizeof(deltas.delta_time));
	memcpy(&deltas.delta_theta[0], buf + offset + 4, sizeof(deltas.delta_theta[0]));
	memcpy(&deltas.delta_theta[1], buf + offset + 8, sizeof(deltas.delta_theta[1]));
	memcpy(&deltas.delta_theta[2], buf + offset + 12, sizeof(deltas.delta_theta[2]));
	memcpy(&deltas.delta_velocity[0], buf + offset + 16, sizeof(deltas.delta_velocity[0]));
	memcpy(&deltas.delta_velocity[1], buf + offset + 20, sizeof(deltas.delta_velocity[1]));
	memcpy(&deltas.delta_velocity[2], buf + offset + 24, sizeof(deltas.delta_velocity[2]));

	return ErrorCode::OK;
}

ErrorCode parse_common_sync_in_cnt(const_bin_buf_ref_t buf, BinaryMessage &msg, CommonSyncInCnt &sync_in_cnt)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, CommonSyncInCnt::GROUP, CommonSyncInCnt::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // SyncInCnt type not found
	}

	memcpy(&sync_in_cnt.sync_in_count, buf + offset + 0, sizeof(sync_in_cnt.sync_in_count));

	return ErrorCode::OK;
}


ErrorCode parse_time_startup(const_bin_buf_ref_t buf, BinaryMessage &msg, TimeStartup &time_startup)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, TimeStartup::GROUP, TimeStartup::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // TimeStartup type not found
	}

	memcpy(&time_startup.time_startup, buf + offset + 0, sizeof(time_startup.time_startup));

	return ErrorCode::OK;
}

ErrorCode parse_time_sync_in(const_bin_buf_ref_t buf, BinaryMessage &msg, TimeSyncIn &time_sync_in)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, TimeSyncIn::GROUP, TimeSyncIn::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // TimeSyncIn type not found
	}

	memcpy(&time_sync_in.time_sync_in, buf + offset + 0, sizeof(time_sync_in.time_sync_in));

	return ErrorCode::OK;
}

ErrorCode parse_sync_in_cnt(const_bin_buf_ref_t buf, BinaryMessage &msg, SyncInCnt &sync_in_cnt)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, SyncInCnt::GROUP, SyncInCnt::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // SyncInCnt type not found
	}

	memcpy(&sync_in_cnt.sync_in_count, buf + offset + 0, sizeof(sync_in_cnt.sync_in_count));

	return ErrorCode::OK;
}

ErrorCode parse_sync_out_cnt(const_bin_buf_ref_t buf, BinaryMessage &msg, SyncOutCnt &sync_out_cnt)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, SyncOutCnt::GROUP, SyncOutCnt::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // SyncOutCnt type not found
	}

	memcpy(&sync_out_cnt.sync_out_count, buf + offset + 0, sizeof(sync_out_cnt.sync_out_count));

	return ErrorCode::OK;
}


ErrorCode parse_imu_status(const_bin_buf_ref_t buf, BinaryMessage &msg, ImuStatus &imu_status)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, ImuStatus::GROUP, ImuStatus::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // IMUStatus type not found
	}

	memcpy(&imu_status, buf + offset, sizeof(imu_status));

	return ErrorCode::OK;
}

ErrorCode parse_uncomp_mag(const_bin_buf_ref_t buf, BinaryMessage &msg, UncompMag &uncomp_mag)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, UncompMag::GROUP, UncompMag::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // UncompMag type not found
	}

	memcpy(&uncomp_mag.uncomp_mag[0], buf + offset + 0, sizeof(uncomp_mag.uncomp_mag[0]));
	memcpy(&uncomp_mag.uncomp_mag[1], buf + offset + 4, sizeof(uncomp_mag.uncomp_mag[1]));
	memcpy(&uncomp_mag.uncomp_mag[2], buf + offset + 8, sizeof(uncomp_mag.uncomp_mag[2]));

	return ErrorCode::OK;
}

ErrorCode parse_uncomp_acc(const_bin_buf_ref_t buf, BinaryMessage &msg, UncompAcc &uncomp_acc)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, UncompAcc::GROUP, UncompAcc::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // UncompAcc type not found
	}

	memcpy(&uncomp_acc.uncomp_acc[0], buf + offset + 0, sizeof(uncomp_acc.uncomp_acc[0]));
	memcpy(&uncomp_acc.uncomp_acc[1], buf + offset + 4, sizeof(uncomp_acc.uncomp_acc[1]));
	memcpy(&uncomp_acc.uncomp_acc[2], buf + offset + 8, sizeof(uncomp_acc.uncomp_acc[2]));

	return ErrorCode::OK;
}

ErrorCode parse_uncomp_gyro(const_bin_buf_ref_t buf, BinaryMessage &msg, UncompGyro &uncomp_gyro)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, UncompGyro::GROUP, UncompGyro::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // UncompGyro type not found
	}

	memcpy(&uncomp_gyro.uncomp_gyro[0], buf + offset + 0, sizeof(uncomp_gyro.uncomp_gyro[0]));
	memcpy(&uncomp_gyro.uncomp_gyro[1], buf + offset + 4, sizeof(uncomp_gyro.uncomp_gyro[1]));
	memcpy(&uncomp_gyro.uncomp_gyro[2], buf + offset + 8, sizeof(uncomp_gyro.uncomp_gyro[2]));

	return ErrorCode::OK;
}

ErrorCode parse_temperature(const_bin_buf_ref_t buf, BinaryMessage &msg, Temperature &temperature)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Temperature::GROUP, Temperature::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Temperature type not found
	}

	memcpy(&temperature.temperature, buf + offset + 0, sizeof(temperature.temperature));

	return ErrorCode::OK;
}

ErrorCode parse_pressure(const_bin_buf_ref_t buf, BinaryMessage &msg, Pressure &pressure)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Pressure::GROUP, Pressure::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Pressure type not found
	}

	memcpy(&pressure.pressure, buf + offset + 0, sizeof(pressure.pressure));

	return ErrorCode::OK;
}

ErrorCode parse_delta_theta(const_bin_buf_ref_t buf, BinaryMessage &msg, DeltaTheta &delta_theta)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, DeltaTheta::GROUP, DeltaTheta::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // DeltaTheta type not found
	}

	memcpy(&delta_theta.delta_time, buf + offset + 0, sizeof(delta_theta.delta_time));
	memcpy(&delta_theta.delta_theta[0], buf + offset + 4, sizeof(delta_theta.delta_theta[0]));
	memcpy(&delta_theta.delta_theta[1], buf + offset + 8, sizeof(delta_theta.delta_theta[1]));
	memcpy(&delta_theta.delta_theta[2], buf + offset + 12, sizeof(delta_theta.delta_theta[2]));

	return ErrorCode::OK;
}

ErrorCode parse_delta_vel(const_bin_buf_ref_t buf, BinaryMessage &msg, DeltaVel &delta_vel)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, DeltaVel::GROUP, DeltaVel::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // DeltaVel type not found
	}

	memcpy(&delta_vel.delta_velocity[0], buf + offset + 0, sizeof(delta_vel.delta_velocity[0]));
	memcpy(&delta_vel.delta_velocity[1], buf + offset + 4, sizeof(delta_vel.delta_velocity[1]));
	memcpy(&delta_vel.delta_velocity[2], buf + offset + 8, sizeof(delta_vel.delta_velocity[2]));

	return ErrorCode::OK;
}

ErrorCode parse_mag(const_bin_buf_ref_t buf, BinaryMessage &msg, Mag &mag)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Mag::GROUP, Mag::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Mag type not found
	}

	memcpy(&mag.mag[0], buf + offset + 0, sizeof(mag.mag[0]));
	memcpy(&mag.mag[1], buf + offset + 4, sizeof(mag.mag[1]));
	memcpy(&mag.mag[2], buf + offset + 8, sizeof(mag.mag[2]));

	return ErrorCode::OK;
}

ErrorCode parse_accel(const_bin_buf_ref_t buf, BinaryMessage &msg, Accel &accel)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Accel::GROUP, Accel::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Accel type not found
	}

	memcpy(&accel.acc[0], buf + offset + 0, sizeof(accel.acc[0]));
	memcpy(&accel.acc[1], buf + offset + 4, sizeof(accel.acc[1]));
	memcpy(&accel.acc[2], buf + offset + 8, sizeof(accel.acc[2]));

	return ErrorCode::OK;
}

ErrorCode parse_angular_rate(const_bin_buf_ref_t buf, BinaryMessage &msg, AngularRate &angular_rate)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, AngularRate::GROUP, AngularRate::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // AngularRate type not found
	}

	memcpy(&angular_rate.gyro[0], buf + offset + 0, sizeof(angular_rate.gyro[0]));
	memcpy(&angular_rate.gyro[1], buf + offset + 4, sizeof(angular_rate.gyro[1]));
	memcpy(&angular_rate.gyro[2], buf + offset + 8, sizeof(angular_rate.gyro[2]));

	return ErrorCode::OK;
}

ErrorCode parse_sens_sat(const_bin_buf_ref_t buf, BinaryMessage &msg, SensSat &sens_sat)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, SensSat::GROUP, SensSat::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // SensSat type not found
	}

	memcpy(&sens_sat, buf + offset, sizeof(sens_sat));

	return ErrorCode::OK;
}


ErrorCode parse_ahrs_status(const_bin_buf_ref_t buf, BinaryMessage &msg, AhrsStatus &ahrs_status)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, AhrsStatus::GROUP, AhrsStatus::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // AhrsStatus type not found
	}

	memcpy(&ahrs_status, buf + offset, sizeof(ahrs_status));

	return ErrorCode::OK;
}

ErrorCode parse_ypr(const_bin_buf_ref_t buf, BinaryMessage &msg, Ypr &ypr)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Ypr::GROUP, Ypr::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Ypr type not found
	}

	memcpy(&ypr.yaw, buf + offset + 0, sizeof(ypr.yaw));
	memcpy(&ypr.pitch, buf + offset + 4, sizeof(ypr.pitch));
	memcpy(&ypr.roll, buf + offset + 8, sizeof(ypr.roll));

	return ErrorCode::OK;
}

ErrorCode parse_quaternion(const_bin_buf_ref_t buf, BinaryMessage &msg, Quaternion &quaternion)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Quaternion::GROUP, Quaternion::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Quaternion type not found
	}

	memcpy(&quaternion.quaternion[0], buf + offset + 0, sizeof(quaternion.quaternion[0]));
	memcpy(&quaternion.quaternion[1], buf + offset + 4, sizeof(quaternion.quaternion[1]));
	memcpy(&quaternion.quaternion[2], buf + offset + 8, sizeof(quaternion.quaternion[2]));
	memcpy(&quaternion.quaternion[3], buf + offset + 12, sizeof(quaternion.quaternion[3]));

	return ErrorCode::OK;
}

ErrorCode parse_dcm(const_bin_buf_ref_t buf, BinaryMessage &msg, Dcm &dcm)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Dcm::GROUP, Dcm::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Dcm type not found
	}

	memcpy(&dcm.dcm, buf + offset, sizeof(dcm.dcm));

	return ErrorCode::OK;
}

ErrorCode parse_mag_ned(const_bin_buf_ref_t buf, BinaryMessage &msg, MagNed &mag_ned)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, MagNed::GROUP, MagNed::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // MagNed type not found
	}

	memcpy(&mag_ned.mag_ned[0], buf + offset + 0, sizeof(mag_ned.mag_ned[0]));
	memcpy(&mag_ned.mag_ned[1], buf + offset + 4, sizeof(mag_ned.mag_ned[1]));
	memcpy(&mag_ned.mag_ned[2], buf + offset + 8, sizeof(mag_ned.mag_ned[2]));

	return ErrorCode::OK;
}

ErrorCode parse_accel_ned(const_bin_buf_ref_t buf, BinaryMessage &msg, AccelNed &accel_ned)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, AccelNed::GROUP, AccelNed::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // AccelNed type not found
	}

	memcpy(&accel_ned.accel_ned[0], buf + offset + 0, sizeof(accel_ned.accel_ned[0]));
	memcpy(&accel_ned.accel_ned[1], buf + offset + 4, sizeof(accel_ned.accel_ned[1]));
	memcpy(&accel_ned.accel_ned[2], buf + offset + 8, sizeof(accel_ned.accel_ned[2]));

	return ErrorCode::OK;
}

ErrorCode parse_lin_body_acc(const_bin_buf_ref_t buf, BinaryMessage &msg, LinBodyAcc &lin_body_acc)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, LinBodyAcc::GROUP, LinBodyAcc::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // LinBodyAcc type not found
	}

	memcpy(&lin_body_acc.lin_body_acc[0], buf + offset + 0, sizeof(lin_body_acc.lin_body_acc[0]));
	memcpy(&lin_body_acc.lin_body_acc[1], buf + offset + 4, sizeof(lin_body_acc.lin_body_acc[1]));
	memcpy(&lin_body_acc.lin_body_acc[2], buf + offset + 8, sizeof(lin_body_acc.lin_body_acc[2]));

	return ErrorCode::OK;
}

ErrorCode parse_lin_accel_ned(const_bin_buf_ref_t buf, BinaryMessage &msg, LinAccelNed &lin_accel_ned)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, LinAccelNed::GROUP, LinAccelNed::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // LinAccelNed type not found
	}

	memcpy(&lin_accel_ned.lin_accel_ned[0], buf + offset + 0, sizeof(lin_accel_ned.lin_accel_ned[0]));
	memcpy(&lin_accel_ned.lin_accel_ned[1], buf + offset + 4, sizeof(lin_accel_ned.lin_accel_ned[1]));
	memcpy(&lin_accel_ned.lin_accel_ned[2], buf + offset + 8, sizeof(lin_accel_ned.lin_accel_ned[2]));

	return ErrorCode::OK;
}

ErrorCode parse_ypr_u(const_bin_buf_ref_t buf, BinaryMessage &msg, YprU &ypr_u)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, YprU::GROUP, YprU::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // YprU type not found
	}

	memcpy(&ypr_u.yaw_u, buf + offset + 0, sizeof(ypr_u.yaw_u));
	memcpy(&ypr_u.pitch_u, buf + offset + 4, sizeof(ypr_u.pitch_u));
	memcpy(&ypr_u.roll_u, buf + offset + 8, sizeof(ypr_u.roll_u));

	return ErrorCode::OK;
}

ErrorCode parse_heave(const_bin_buf_ref_t buf, BinaryMessage &msg, Heave &heave)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, Heave::GROUP, Heave::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // Heave type not found
	}

	memcpy(&heave.heave, buf + offset + 0, sizeof(heave.heave));
	memcpy(&heave.heave_rate, buf + offset + 4, sizeof(heave.heave_rate));
	memcpy(&heave.delayed_heave, buf + offset + 8, sizeof(heave.delayed_heave));

	return ErrorCode::OK;
}

ErrorCode parse_att_u(const_bin_buf_ref_t buf, BinaryMessage &msg, AttU &att_u)
{
	msg::len_t offset = 0;

	if ((get_offset_by_type(buf, msg, offset, AttU::GROUP, AttU::TYPE) != ErrorCode::OK)) {
		return ErrorCode::InvalidParameter; // AttU type not found
	}

	memcpy(&att_u.att_u, buf + offset, sizeof(att_u.att_u));

	return ErrorCode::OK;
}


} // namespace bin

} // namespace vn
