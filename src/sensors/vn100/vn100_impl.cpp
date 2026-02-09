/**
 * @file vn100_impl.cpp
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

void VN100::loop()
{
	// FIX: timing on this run loop is not necessary, but you shouldn't call it faster than 1kHz
	// Main loop logic here

	if (!_initialized) {
		if (!init()) {
			Serial.println("Failed to initialize VN100. Trying again next loop...");
			return;
		}
	}

	_uart.loop();
}

void VN100::binary_message_callback(vn::bin::const_bin_buf_ref_t buf, vn::msg::len_t length,
		vn::bin::BinaryMessage &msg)
{
	VN100 *instance = &vn100;

	// FIX: the following parsing logic is an example. you will need to adjust it depending on what messages you want to parse and publish.

	// parse out and publish message 1
	if (vn::bin::compare_messages(instance->bin_msg_1, msg) == vn::ErrorCode::OK) {
		vn::bin::TimeStartup time_startup;

		if (vn::bin::parse_time_startup(buf, msg, time_startup) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse time startup from binary message");
			return;
		}

		vn::bin::Accel accel;

		if (vn::bin::parse_accel(buf, msg, accel) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse accel from binary message");
			return;
		}

		vn::bin::AngularRate angular_rate;

		if (vn::bin::parse_angular_rate(buf, msg, angular_rate) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse angular rate from binary message");
			return;
		}

		vn::bin::SensSat sens_sat;

		if (vn::bin::parse_sens_sat(buf, msg, sens_sat) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse sensor saturation from binary message");
			return;
		}

		// static uint32_t last_angular_rate_print_time_us = 0;
		// const uint32_t now_us = micros();
		//
		// if (now_us - last_angular_rate_print_time_us >= 100000) {
		// 	last_angular_rate_print_time_us = now_us;
		// 	Serial.printf("VN100 gyro [rad/s]: x=%.3f y=%.3f z=%.3f\n", angular_rate.gyro[0], angular_rate.gyro[1], angular_rate.gyro[2]);
		// }

		// publish this data here

		return;
	}

	// parse out and publish message 2
	if (vn::bin::compare_messages(instance->bin_msg_2, msg) == vn::ErrorCode::OK) {
		vn::bin::TimeStartup time_startup;

		if (vn::bin::parse_time_startup(buf, msg, time_startup) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse time startup from binary message");
			return;
		}

		vn::bin::Temperature temperature;

		if (vn::bin::parse_temperature(buf, msg, temperature) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse temperature from binary message");
			return;
		}

		vn::bin::Pressure pressure;

		if (vn::bin::parse_pressure(buf, msg, pressure) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse pressure from binary message");
			return;
		}

		vn::bin::Mag mag;

		if (vn::bin::parse_mag(buf, msg, mag) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse magnetometer from binary message");
			return;
		}

		vn::bin::Ypr ypr;

		if (vn::bin::parse_ypr(buf, msg, ypr) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse ypr from binary message");
			return;
		}

		vn::bin::Quaternion quaternion;

		if (vn::bin::parse_quaternion(buf, msg, quaternion) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse quaternion from binary message");
			return;
		}

		vn::bin::LinAccelNed lin_accel_ned;

		if (vn::bin::parse_lin_accel_ned(buf, msg, lin_accel_ned) != vn::ErrorCode::OK) {
			Serial.println("Failed to parse linear acceleration NED from binary message");
			return;
		}

		static uint32_t last_ypr_print_time_us = 0;
		const uint32_t now_us = micros();

		if (now_us - last_ypr_print_time_us >= 100000) {
			last_ypr_print_time_us = now_us;
			Serial.printf("YPR [deg] | yaw: %+8.2f | pitch: %+8.2f | roll: %+8.2f\n",
				      ypr.yaw, ypr.pitch, ypr.roll);
		}

		return;
	}

	// parse out and publish message 3
	if (vn::bin::compare_messages(instance->bin_msg_3, msg) == vn::ErrorCode::OK) {
		return;
	}
}
