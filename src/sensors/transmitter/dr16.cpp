#include "dr16.hpp"
#include <Arduino.h>
#include "sensors/RefSystem.hpp"
#include "comms/data/sendable.hpp"

DR16::DR16(const Cfg::DR16& config_) : config(config_) {

}

void DR16::init() {
	// start hardware_serial_port HardwareSerial with 1
	Serial8.begin(100000, SERIAL_8E1_RXINV_TXINV);
	// await any active writing and clear the buffer
	Serial8.flush();
	Serial8.clear();

	// init all input buffers to 0
	for (int i = 0; i < DR16_PACKET_SIZE; i++) {
		m_inputRaw[i] = 0;
	}

	for (int i = 0; i < DR16_INPUT_VALUE_COUNT; i++) {
		m_input[i] = 0;
	}
}

void DR16::read() {
	// each channel is 11 bits, minus the switches and keyboard inputs
	uint16_t c0{ 0 }, c1{ 0 }, c2{ 0 }, c3{ 0 }, wh{ 0 };
	uint8_t s1{ 0 }, s2{ 0 }, k1{ 0 }, k2{ 0 };

	// if read is not called regularly enough, it may miss a packet and start reading halfway through another packet
	// since the dr16 data structure does not include a start of transmission byte, we need to time our reads based on
	// it's data stream. 

	// the dr16 gives us data in 18 byte packets at 100kbps. Including UART bloat, that comes out to be around 120us per byte. 
	// however, the dr16 also pauses in between packets for about 11-12ms. 

	// this alignment code polls to find this packet break, indicated by not receiving a byte for longer than DR16_ALIGNMENT_LONG_INTERVAL_THRESHOLD
	// once it finds it, it clears the buffer, marks the controller as disconnected (for safety), and returns. Allowing the next 
	// read to catch the incoming packet successfully
	if (Serial8.available() > DR16_PACKET_SIZE * 4) {
		uint32_t align_start = micros();
		// wait for a break in the data transmission
		Serial8.clear();

		// number of byte intervals the alignment has encountered
		int interval_count = 0;

		// whether the alignment was successful or stopped prematurely due to a timeout
		bool alignment_timed_out = true;

		// wait until a long interval
		uint32_t long_interval_start = micros();
		while (micros() - long_interval_start < DR16_ALIGNMENT_TIMEOUT) {
			interval_count++;
			
			// keep track of the this interval's start time and how many bytes were in the serial buffer
			uint32_t start = micros();
			int last_available = Serial8.available();

			// poll until we get a byte or the long interval threshold time is reached
			while (last_available == Serial8.available() && micros() - start < DR16_ALIGNMENT_LONG_INTERVAL_THRESHOLD);
			uint32_t end = micros();
			
			Serial.printf("DR16: Still aligning (%d)\n", interval_count);

			// if this interval was a long interval (break in packets), call the alignment done and finish up
			// also mark this as a successful alignment, rather than it timing out
			if (end - start >= DR16_ALIGNMENT_LONG_INTERVAL_THRESHOLD) {
				alignment_timed_out = false;
				break;
			};
		}

		// print success or failure
		if (alignment_timed_out) {
			Serial.printf("DR16: Alignment timed out, trying again next loop\n\n");
		} else {
			uint32_t align_end = micros();
			Serial.printf("DR16: Aligned successfully\n");
			Serial.printf("DR16: Alignment took %fms\n\n", (align_end - align_start) / 1000.f);
		}

		// clear the buffer to get ready for the next packet
		Serial8.clear();

		// mark the controller as disconnected since we have not yet received valid data
		m_connected = false;

		// no need to do more reading logic since the buffer is now empty		
		return;
	}


	// dont read if there are less than 18 bytes, i.e. we caught the packet as it was being written
	if (Serial8.available() < DR16_PACKET_SIZE) {
		if (millis() - m_disctTime > 250) {
			m_connected = false;
		}
		return;
	}

	m_disctTime = millis();
	m_connected = true;

	  // issue read command, fills m_inputRaw with 18 bytes
	Serial8.readBytes(m_inputRaw, DR16_PACKET_SIZE);

	// set channel values, since each channel is packed within each other, and are 11 bits long
	// some bit shifting is required
	c0 = ((m_inputRaw[1] & 0x07) << 8) | m_inputRaw[0];
	c1 = ((m_inputRaw[2] & 0x3f) << 5) | ((m_inputRaw[1] & 0xf8) >> 3);
	c2 = ((m_inputRaw[4] & 0x01) << 10) | ((m_inputRaw[3] & 0xff) << 2) | ((m_inputRaw[2] & 0xc0) >> 6);
	c3 = ((m_inputRaw[5] & 0x0f) << 7) | ((m_inputRaw[4] & 0xfe) >> 1);
	wh = ((m_inputRaw[17] & 0x7) << 8) | m_inputRaw[16];
	k1 = m_inputRaw[14];
	k2 = m_inputRaw[15];
  
	mouse_x = (m_inputRaw[7] << 8) | m_inputRaw[6]; 
	mouse_y = (m_inputRaw[9] << 8) | m_inputRaw[8]; 
	mouse_z = (m_inputRaw[11] << 8) | m_inputRaw[10];
	l_mouse_button = m_inputRaw[12];
	r_mouse_button = m_inputRaw[13];
	s1 = (m_inputRaw[5] & 0x30) >> 4;
	s2 = (m_inputRaw[5] & 0xc0) >> 6;

	// set these split values into the seperated raw input array
	m_inputRawSeperated[0] = c0;
	m_inputRawSeperated[1] = c1;
	m_inputRawSeperated[2] = c2;
	m_inputRawSeperated[3] = c3;
	m_inputRawSeperated[4] = wh;
	m_inputRawSeperated[5] = s1;
	m_inputRawSeperated[6] = s2;

	// simple safety check
	if (is_data_valid()) {
		m_fail = false;
		m_failTime = 0;
		// assign formated data (within ranges of [-1,1]) to the true input buffer
		// joy sticks
		m_input[0] = bounded_map(c0, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
		m_input[1] = bounded_map(c1, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
		m_input[2] = bounded_map(c2, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
		m_input[3] = bounded_map(c3, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;

		// wheel
		m_input[4] = bounded_map(wh, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;

		// switches
		m_input[5] = (float)s1;
		m_input[6] = (float)s2;

		/// data from the rm control client
		// first byte
		keys.w = (k1 >> 0) & 0x01;
		keys.s = (k1 >> 1) & 0x01;
		keys.a = (k1 >> 2) & 0x01;
		keys.d = (k1 >> 3) & 0x01;
		keys.shift = (k1 >> 4) & 0x01;
		keys.ctrl = (k1 >> 5) & 0x01;
		keys.q = (k1 >> 6) & 0x01;
		keys.e = (k1 >> 7) & 0x01;
		// second byte
		keys.r = (k2 >> 0) & 0x01;
		keys.f = (k2 >> 1) & 0x01;
		keys.g = (k2 >> 2) & 0x01;
		keys.z = (k2 >> 3) & 0x01;
		keys.x = (k2 >> 4) & 0x01;
		keys.c = (k2 >> 5) & 0x01;
		keys.v = (k2 >> 6) & 0x01;
		keys.b = (k2 >> 7) & 0x01;

		mode_changed_flag = (get_l_switch() != 	prev_l_switch_pos);
		prev_l_switch_pos = get_l_switch();
	} else {
		uint32_t dt = micros() - m_prevTime;
		m_failTime += dt;
		if (m_failTime > DR16_FAIL_STATE_TIMEOUT)
			m_fail = true;
		else
			m_fail = false;
	}

	m_prevTime = micros();
}

void DR16::zero() {
	// zero input buffer
	for (int i = 0; i < DR16_INPUT_VALUE_COUNT; i++) {
		m_input[i] = 0;
	}

	// set switches to a specific value
	m_input[5] = 1;
	m_input[6] = 1;
}

float* DR16::get_input() {
	return m_input;
}

void DR16::print() {
	Serial.printf("RStick X: %.2f\tRStick Y: %.2f\tLStick X: %.2f\tLStick Y: %.2f\tWheel: %.2f\tRSwitch: %.2f\tLSwitch: %.2f\n", m_input[0], m_input[1], m_input[2], m_input[3], m_input[4], m_input[5], m_input[6]);
}

void DR16::print_raw() {
	for (int i = 0; i < DR16_PACKET_SIZE; i++)
		Serial.printf("%.2x\t", m_inputRaw[i]);
	Serial.println();
}

float DR16::bounded_map(int value, int in_low, int in_high, int out_low, int out_high) {
	// this is derived from sthe arduino map() function
	value = max(min(value, in_high), in_low);

	return (float)(value - in_low) * (out_high - out_low) / (float)(in_high - in_low) + out_low;
}

bool DR16::is_data_valid() {
	// go through all values in raw seperated input and compare them against maximum and minimum values
	// the - 2 is to exclude switch values
	for (int i = 0; i < DR16_INPUT_VALUE_COUNT - 2; i++) {
		if (m_inputRawSeperated[i] < DR16_CONTROLLER_INPUT_LOW || m_inputRawSeperated[i] > DR16_CONTROLLER_INPUT_HIGH)
			return false;
	}

	return true;
}

float DR16::get_r_stick_x() {
	return m_input[0];
}

float DR16::get_r_stick_y() {
	return m_input[1];
}

float DR16::get_l_stick_x() {
	return m_input[2];
}

float DR16::get_l_stick_y() {
	return m_input[3];
}

float DR16::get_wheel() {
	return m_input[4];
}

SwitchPos DR16::get_r_switch() {
	return static_cast<SwitchPos> (m_input[5]);
}

SwitchPos DR16::get_l_switch() {
	return static_cast<SwitchPos> (m_input[6]);
}

int DR16::get_mouse_y() {
	return mouse_y;
}

int DR16::get_mouse_x() {
	return mouse_x;
}

int DR16::get_mouse_z() {
	return mouse_z;
}

bool DR16::get_l_mouse_button() {
	return l_mouse_button;
}

bool DR16::get_r_mouse_button() {
	return r_mouse_button;
}

void DR16::send_to_comms() {
	Comms::Sendable<DR16Data> dr16_data;
	dr16_data.data = get_dr16_data();
	dr16_data.send_to_comms();
}

bool DR16::is_safety_mode() {
	// safety mode is when the right switch is in the backward position
	return get_l_switch() == SwitchPos::FORWARD || !is_data_valid() || !is_connected();
}

bool DR16::is_hive_mode() {
	// hive mode is when the right switch is in the middle position
	return get_l_switch() == SwitchPos::BACKWARD;
}

bool DR16::is_teensy_mode() {
	// teensy mode is when the right switch is in the forward position
	return get_l_switch() == SwitchPos::MIDDLE;
}

bool DR16::mode_changed() {
	return mode_changed_flag;
}

DR16Data DR16::get_dr16_data(){
	DR16Data dr16_data;
	dr16_data.mouse_x = mouse_x;
	dr16_data.mouse_y = mouse_y;
	dr16_data.mouse_z = mouse_z;
	dr16_data.l_mouse_button = l_mouse_button;
	dr16_data.r_mouse_button = r_mouse_button;
	dr16_data.l_switch = get_l_switch();
	dr16_data.r_switch = get_r_switch();
	dr16_data.l_stick_x = get_l_stick_x();
	dr16_data.l_stick_y = get_l_stick_y();
	dr16_data.r_stick_x = get_r_stick_x();
	dr16_data.r_stick_y = get_r_stick_y();
	dr16_data.wheel = get_wheel();
	dr16_data.keys = keys;
	return dr16_data;
}

void DR16::manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, bool not_safety_mode, float& feed, float& last_feed) {
	bool has_lower_feeder = estimated_state_map.get_state_map().find(Cfg::StateName::LowerFeeder) != estimated_state_map.get_state_map().end();

	float delta = control_input_timer.delta();
	transmitter_pos_x += mouse_x * 0.05 * delta;
	transmitter_pos_y += mouse_y * 0.05 * delta;

	vtm_pos_x += ref.ref_data.kbm_interaction.mouse_speed_x * 0.05 * delta;
	vtm_pos_y += ref.ref_data.kbm_interaction.mouse_speed_y * 0.05 * delta;

	float pitch_min = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.min;
    float pitch_max = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.max;
    float pitch_average = 0.5 * (pitch_min + pitch_max);
    pitch_min -= pitch_average;
    pitch_max -= pitch_average;

	// clamp to pitch limits
	if (transmitter_pos_y < pitch_min) {
		transmitter_pos_y = pitch_min;
	}
	if (transmitter_pos_y > pitch_max) {
		transmitter_pos_y = pitch_max;
	}
	if (vtm_pos_y < pitch_min) {
		vtm_pos_y = pitch_min;
	}
	if (vtm_pos_y > pitch_max) {
		vtm_pos_y = pitch_max;
	}

	float chassis_vel_x = 0;
	float chassis_vel_y = 0;
	float chassis_pos_x = 0;
	float chassis_pos_y = 0;

	if (estimated_state_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Velocity) { // if we should be controlling velocity

		chassis_vel_x = get_l_stick_y() * 5.4 +
						(-ref.ref_data.kbm_interaction.key_w + ref.ref_data.kbm_interaction.key_s) * 2.5;

		chassis_vel_x += (-keys.w + keys.s) * 2.5;

		chassis_vel_y = -(get_l_stick_x() * 5.4) +
						(ref.ref_data.kbm_interaction.key_d - ref.ref_data.kbm_interaction.key_a) * 2.5;

		chassis_vel_y += (keys.d - keys.a) * 2.5;
		
	} else if (estimated_state_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Position) { // if we should be controlling position
		chassis_pos_x = get_l_stick_x() * 2 + pos_offset_x;
		chassis_pos_y = get_l_stick_y() * 2 + pos_offset_y;
	}

	float chassis_spin = get_wheel() * 25;
	float pitch_target = 1.57 + -get_r_stick_y() * 0.3 + vtm_pos_y;
	float yaw_target = -get_r_stick_x() * 1.5 - vtm_pos_x;

	float fly_wheel_target =
		(get_r_switch() == SwitchPos::FORWARD || get_r_switch() == SwitchPos::MIDDLE) ? 18 : 0; // m/s
	// if the right switch is forward, and either the left mouse button is pressed or the right switch is not
	// backward, set the feeder to something. Otherwise, set it to 0
	float feeder_target = (((ref.ref_data.kbm_interaction.button_left) &&
							get_r_switch() != SwitchPos::BACKWARD) || get_r_switch() == SwitchPos::FORWARD) ? 10 : 0;
	if (estimated_state_map[Cfg::StateName::Feeder].config().governor_type == Cfg::StateOrder::Position) {
		float dt2 = timer.delta();
		if (dt2 > 0.1)
			dt2 = 0;
		// check if the shooter is active
		if (not_safety_mode && ref.ref_data.robot_performance.shooter_power_active)
			feed += feeder_target * dt2;
		
		if (has_lower_feeder) {
			target_state_map[Cfg::StateName::Feeder].set_position(estimated_state_map[Cfg::StateName::LowerFeeder].get_position());
			target_state_map[Cfg::StateName::LowerFeeder].set_position((int)feed);
		} else {
			target_state_map[Cfg::StateName::Feeder].set_position((int)feed);
		}
	} else { 
		target_state_map[Cfg::StateName::Feeder].set_velocity(feeder_target);
	}
	// if (transmitter->get_r_switch() == 1 && last_switch != 1) {
	//     feed++;
	// }
	// last_switch = transmitter->get_r_switch();
	// set manual controls
	target_state_map[Cfg::StateName::ChassisX].set_position(chassis_pos_x);
	target_state_map[Cfg::StateName::ChassisX].set_velocity(chassis_vel_x);
	target_state_map[Cfg::StateName::ChassisY].set_position(chassis_pos_y);
	target_state_map[Cfg::StateName::ChassisY].set_velocity(chassis_vel_y);
	target_state_map[Cfg::StateName::ChassisHeading].set_velocity(chassis_spin);
	target_state_map[Cfg::StateName::GimbalYaw].set_position(yaw_target);
	target_state_map[Cfg::StateName::GimbalYaw].set_velocity(0);
	target_state_map[Cfg::StateName::GimbalPitch].set_position(pitch_target);
	target_state_map[Cfg::StateName::GimbalPitch].set_velocity(0);
	target_state_map[Cfg::StateName::Flywheels].set_velocity(fly_wheel_target);

	// when in teensy control mode reset hive toggle
	if (is_teensy_mode() && mode_changed()) {
		pos_offset_x = estimated_state_map[Cfg::StateName::ChassisX].get_position();
		pos_offset_y = estimated_state_map[Cfg::StateName::ChassisY].get_position();
		feed = last_feed;
	}
}