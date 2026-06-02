#include "ET16S.hpp"
#include "sensors/RefSystem.hpp"
#include "comms/data/sendable.hpp"
#include "state.hpp"

ET16S::ET16S(const Cfg::ET16S& config) : config(config) { }

void ET16S::init() {
	Serial8.begin(100000, SERIAL_8E1_RXINV_TXINV);
	Serial8.flush();
	Serial8.clear();
	//configure safety switch;
	channel[4].kind = InputKind::THREE_SWITCH;
	channel[4].id = ChannelId::SWITCH_A;
	//Turn safety on
	channel[4].data = static_cast<float>(SwitchPos::FORWARD);

	//configure sticks
	//right stick horizontal
	channel[r_stick_x_num].kind = InputKind::STICK;
	//right stick vertical
	channel[1].kind = InputKind::STICK;
	//left stick vertical
	channel[2].kind = InputKind::STICK;
	//right stick horizontal
	channel[3].kind = InputKind::STICK;
	//configure remaining channels
	set_config();
}

void ET16S::read() {
	static uint8_t last_byte = 0xAA;
	
	// if we have missed more than 4 packets (for example on startup), clear the buffer and start fresh
	if (Serial8.available() > ET16S_PACKET_SIZE * 4) {
		Serial8.clear();
	}

	// only start the "peeking" process if we have at least 2 packets worth of data, therefore we can afford to skip some bytes
	if (Serial8.available() < ET16S_PACKET_SIZE * 2) {
		return;
	}

	// try to find the start of the frame. This is achieved by seeing if the current byte to read is 0x0f and the previous byte read was 0x00
	// if this is the case, we have found the start of the frame
	while (Serial8.peek() != 0x0f) {
		last_byte = Serial8.read();
	}

	// if the last byte was not 0x00, we have not found the start of the frame
	if (last_byte != 0x00) {
		return;
	}

	// if we have less than a full packet available, we cannot read the full frame
	// however we are "aligned" with the start of the frame, so we can wait for the next frame
	if (Serial8.available() < ET16S_PACKET_SIZE) {
		return;
	}

	// read the full frame
	for (int i = 0; i < ET16S_PACKET_SIZE; i++) {
		m_inputRaw[i] = Serial8.read();
	}

	//format raw data
	format_raw(m_inputRaw);
	//set flag data
	channel[16].data = channel[16].raw_format;
	//set remaining data
	set_channel_data();
	//Check flag byte for disconnect
	test_connection();

	mode_changed_flag = (get_safety_switch() != prev_safety_switch_pos);
	prev_safety_switch_pos = get_safety_switch();
}

void ET16S::print() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT; i++) {
		Serial.printf("%f ", channel[i].data);
	}

	Serial.println();
}

void ET16S::print_raw() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT; i++) {
		Serial.printf("%.3u ", channel[i].raw_format);
	}

	Serial.println();
}

void ET16S::print_raw_bin(uint8_t m_inputRaw[ET16S_PACKET_SIZE]) {
	for (int i = 0; i < ET16S_PACKET_SIZE; i++) {
		for (int ii = 0; ii <= 7; ii++) {
			int k = m_inputRaw[i] >> ii;
			if (k & 1)
				Serial.print("1");
			else
				Serial.print("0");
		}

		Serial.printf(" ");
	}

	Serial.println();
}

void ET16S::print_format_bin(int channel_num) {
	if (channel_num > ET16S_INPUT_VALUE_COUNT || channel_num < 0) {
		Serial.print("Invalid channel used for print_format_bin. Must be 0-16");
		return;
	}

	for (int ii = 0; ii <= 16; ii++) {
		int k = channel[channel_num].raw_format >> ii;
		if (k & 1)
			Serial.print("1");
		else
			Serial.print("0");
	}

	Serial.printf(" ");
	Serial.println();
}

void ET16S::format_raw(uint8_t m_inputRaw[ET16S_PACKET_SIZE]) {
	uint16_t c[23] = { 0 };

	//Channel 1
	c[0] = m_inputRaw[1];
	c[1] = m_inputRaw[2] << 5;
	channel[0].raw_format = ((c[0] << 5) | (c[1] << 8));
	channel[0].raw_format >>= 5;

	//Channel 2
	c[1] = m_inputRaw[2] >> 3;
	c[2] = m_inputRaw[3] << 2;
	channel[1].raw_format = (c[1] << 5) | (c[2] << 8);
	channel[1].raw_format >>= 5;

	//Channel 3
	c[2] = m_inputRaw[3] >> 6;
	c[3] = m_inputRaw[4];
	c[4] = m_inputRaw[5] << 7;
	channel[2].raw_format = (c[2] << 5) | (c[3] << 7) | (c[4] << 8);
	channel[2].raw_format >>= 5;

	//Channel 4
	c[4] = m_inputRaw[5] >> 1;
	c[5] = m_inputRaw[6] << 4;
	channel[3].raw_format = (c[4] << 5) | (c[5] << 8);
	channel[3].raw_format >>= 5;

	//Channel 5
	c[5] = m_inputRaw[6] >> 4;
	c[6] = m_inputRaw[7] << 1;
	channel[4].raw_format = (c[5] << 5) | (c[6] << 8);
	channel[4].raw_format >>= 5;

	//Channel 6
	c[6] = m_inputRaw[7] >> 7;
	c[7] = m_inputRaw[8];
	c[8] = m_inputRaw[9] << 6;
	channel[5].raw_format = (c[6] << 5) | (c[7] << 6) | (c[8] << 8);
	channel[5].raw_format >>= 5;

	//Channel 7
	c[8] = m_inputRaw[9] >> 2;
	c[9] = m_inputRaw[10] << 3;
	channel[6].raw_format = (c[8] << 5) | (c[9] << 8);
	channel[6].raw_format >>= 5;

	//Channel 8
	c[9] = m_inputRaw[10] >> 5;
	c[10] = m_inputRaw[11];
	channel[7].raw_format = (c[9] << 5) | (c[10] << 8);
	channel[7].raw_format >>= 5;

	//Channel 9
	c[11] = m_inputRaw[12];
	c[12] = m_inputRaw[13] << 5;
	channel[8].raw_format = (c[11] << 5) | (c[12] << 8);
	channel[8].raw_format >>= 5;

	//Channel 10
	c[12] = m_inputRaw[13] >> 3;
	c[13] = m_inputRaw[14] << 2;
	channel[9].raw_format = (c[12] << 5) | (c[13] << 8);
	channel[9].raw_format >>= 5;

	//Channel 11
	c[13] = m_inputRaw[14] >> 6;
	c[14] = m_inputRaw[15];
	c[15] = m_inputRaw[16] << 7;
	channel[10].raw_format = (c[13] << 5) | (c[14] << 7) | (c[15] << 8);
	channel[10].raw_format >>= 5;

	//Channel 12
	c[15] = m_inputRaw[16] >> 1;
	c[16] = m_inputRaw[17] << 4;
	channel[11].raw_format = (c[15] << 5) | (c[16] << 8);
	channel[11].raw_format >>= 5;

	//Channel 13
	c[16] = m_inputRaw[17] >> 4;
	c[17] = m_inputRaw[18] << 1;
	channel[12].raw_format = (c[16] << 5) | (c[17] << 8);
	channel[12].raw_format >>= 5;

	//Channel 14
	c[17] = m_inputRaw[18] >> 7;
	c[18] = m_inputRaw[19];
	c[19] = m_inputRaw[20] << 6;
	channel[13].raw_format = (c[17] << 5) | (c[18] << 6) | (c[19] << 8);
	channel[13].raw_format >>= 5;

	//Channel 15
	c[19] = m_inputRaw[20] >> 2;
	c[20] = m_inputRaw[21] << 3;
	channel[14].raw_format = (c[19] << 5) | (c[20] << 8);
	channel[14].raw_format >>= 5;

	//Channel 16
	c[20] = m_inputRaw[21] >> 5;
	c[21] = m_inputRaw[22];
	channel[15].raw_format = (c[20] << 5) | (c[21] << 8);
	channel[15].raw_format >>= 5;

	//channel 17 (Error Flag)
	c[22] = m_inputRaw[23];
	channel[16].raw_format = (c[22] << 5);
	channel[16].raw_format >>= 5;
}

float ET16S::map_raw(InputChannel input) {
	float val = input.raw_format;
	InputKind kind = input.kind;

	switch (kind) {
	case InputKind::STICK: {
		val = map_math(val,-1,1);
		break;
	}
	case InputKind::TWO_SWITCH: {
		if (val == max_in) {
			val = static_cast<float>(SwitchPos::BACKWARD);
		} else {
			val = static_cast<float>(SwitchPos::FORWARD);
		}
		break;
	}
	case InputKind::THREE_SWITCH: {
		if (val == max_in) {
			val = static_cast<float>(SwitchPos::BACKWARD);
		} else if (val == min_in) {
			val = static_cast<float>(SwitchPos::FORWARD);
		} else {
			val = static_cast<float>(SwitchPos::MIDDLE);
		}
		break;
	}
	case InputKind::DIAL: {
		//Dial values go towards -1 as it is spun left
		val = -map_math(val,-1,1);

		if (val == -0.f) { val = 0; } //used to prevent -0 float
		break;
	}
	case InputKind::SLIDER: {
		//Wheel values go towards -1 as it is spun down
		val = -map_math(val,-1,1);

		if (val == -0.f) { val = 0; } //used to prevent -0 float
		break;
	}
	case InputKind::TRIM: {
		break;
	}
	case InputKind::INVALID: {
		break;
	}
	case InputKind::FLAG: {
		break;
	}
	}

	return val;
}

float ET16S::map_math(float val,float min_out, float max_out){
	// maps input value from -1 to 1
	val = min_out + (val - min_in) * (max_out - min_out) / (max_in - min_in);
	//ensure val is in range 
	if(val>max_out){
		val=max_out;
	}
	else if (val<min_out){
		val=min_out;
	}
	return val;
}

void ET16S::set_config() {
	//Valid channel types include STICK,TWO_SWITCH_THREE_SWITCH,
	//DIAL,SLIDER,TRIM,FLAG,INVALID
	//note (trim is not mapped)
	channel[r_stick_x_num].id = ChannelId::R_STICK_X;
	channel[r_stick_y_num].id = ChannelId::R_STICK_Y;
	channel[l_stick_x_num].id = ChannelId::L_STICK_X;
	channel[l_stick_y_num].id = ChannelId::L_STICK_Y;
	
	channel[switch_b_num].id = ChannelId::SWITCH_B;
	channel[switch_c_num].id = ChannelId::SWITCH_C;
	channel[switch_d_num].id = ChannelId::SWITCH_D;
	channel[switch_e_num].id = ChannelId::SWITCH_E;
	channel[switch_f_num].id = ChannelId::SWITCH_F;
	channel[switch_g_num].id = ChannelId::SWITCH_G;
	channel[switch_h_num].id = ChannelId::SWITCH_H;
	channel[r_slider_num].id = ChannelId::R_SLIDER;
	channel[l_slider_num].id = ChannelId::L_SLIDER;
	channel[r_dial_num].id = ChannelId::R_DIAL;
	channel[l_dial_num].id = ChannelId::L_DIAL;
	channel[16].id = ChannelId::FLAG;

	for (int i = 5; i < ET16S_INPUT_VALUE_COUNT; i++){
		ChannelId id = channel[i].id;
		switch(id){
		case ChannelId::L_STICK_X:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::L_STICK_Y:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::R_STICK_X:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::R_STICK_Y:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::L_DIAL:
			channel[i].kind = InputKind::DIAL;
			break;
		case ChannelId::R_DIAL:
			channel[i].kind = InputKind::DIAL;
			break;
		case ChannelId::SWITCH_A:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_B:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_C:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_D:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_E:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_F:
			channel[i].kind = InputKind::TWO_SWITCH;
			break;
		case ChannelId::SWITCH_G:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_H:
			channel[i].kind = InputKind::TWO_SWITCH;
			break;
		case ChannelId::L_SLIDER:
			channel[i].kind = InputKind::SLIDER;
			break;
		case ChannelId::R_SLIDER:
			channel[i].kind = InputKind::SLIDER;
			break;
		case ChannelId::TRIM_1:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_2:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_3:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_4:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_5:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_6:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::UNMAPPED:
			channel[i].kind = InputKind::INVALID;
			break;
		case ChannelId::FLAG:
			channel[i].kind = InputKind::FLAG;
			break;
			
	   }
	}
}

float ET16S::average_channel(float sample, int channel_index) {
	// set the sample
	// circular buffer 
	int channel_value_index = value_indices[channel_index];
	channel_values_circular_buf[channel_index][channel_value_index] = sample;
	value_indices[channel_index] = (channel_value_index + 1) % AVERAGE_SAMPLE_COUNT;
	
	float sum = 0;
	float average = 0;
	for (int i = 0; i < AVERAGE_SAMPLE_COUNT; i++) {
		sum += channel_values_circular_buf[channel_index][i];
	}

	average = sum / AVERAGE_SAMPLE_COUNT;
	return average;
}

void ET16S::set_channel_data() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT;i++) {
		// we only want to average the stick, dial, and slider channels
		// the transmitter's output for continuous data is a bit noisy, enough to move the yaw back and forth
		// we average the last few samples to smoothen the output
		if (channel[i].kind == InputKind::STICK || channel[i].kind == InputKind::DIAL || channel[i].kind == InputKind::SLIDER) {
			channel[i].data = average_channel(map_raw(channel[i]), i);
		} else {
			channel[i].data = map_raw(channel[i]);
		}
	}
}

void ET16S::test_connection() {		
	uint16_t flag_byte = channel[16].raw_format;
	if (flag_byte & DISCONNECT) {
		is_connect = false;
	} else {
		is_connect = true;
	}
}

SwitchPos ET16S::get_safety_switch() {
	return static_cast<SwitchPos> (channel[4].data);
}

float ET16S::get_r_stick_x() {
	return channel[r_stick_x_num].data;
}

float ET16S::get_r_stick_y() {
	return channel[r_stick_y_num].data;
}

float ET16S::get_l_stick_x() {
	return channel[l_stick_x_num].data;
}

float ET16S::get_l_stick_y() {
	return channel[l_stick_y_num].data;
}

SwitchPos ET16S::get_switch_b(){
	return static_cast<SwitchPos> (channel[switch_b_num].data);
}

SwitchPos ET16S::get_switch_c(){
	return static_cast<SwitchPos> (channel[switch_c_num].data);
}

SwitchPos ET16S::get_switch_d(){
	return static_cast<SwitchPos> (channel[switch_d_num].data);
}

SwitchPos ET16S::get_switch_e(){
	return static_cast<SwitchPos> (channel[switch_e_num].data);
}

SwitchPos ET16S::get_switch_f(){
	return static_cast<SwitchPos> (channel[switch_f_num].data);
}

SwitchPos ET16S::get_switch_g(){
	return static_cast<SwitchPos> (channel[switch_g_num].data);
}

SwitchPos ET16S::get_switch_h(){
	return static_cast<SwitchPos> (channel[switch_h_num].data);
}

float ET16S::get_l_slider(){
	return channel[l_slider_num].data;
}

float ET16S::get_r_slider(){
	return channel[r_slider_num].data;
}

float ET16S::get_trim_one(){
	return channel[trim_one_num].data;
}

float ET16S::get_trim_two(){
	return channel[trim_two_num].data;
}

float ET16S::get_trim_three(){
	return channel[trim_three_num].data;
}

float ET16S::get_trim_four(){
	return channel[trim_four_num].data;
}

float ET16S::get_trim_five(){
	return channel[trim_five_num].data;
}

float ET16S::get_trim_six(){
	return channel[trim_six_num].data;
}

float ET16S::get_l_dial(){
	return channel[l_dial_num].data;
}

float ET16S::get_r_dial(){
	return channel[r_dial_num].data;
}

float ET16S::get_channel_data(int chan_num){
	// Will return nothing if an incorrect channel  number is given
	if ((chan_num < 0) || (chan_num > 16)){		
		return {};
	}
	return channel[chan_num].data;
}

bool ET16S::is_connected() {
	return is_connect;
}

SwitchPos ET16S::get_l_switch(){
	return static_cast<SwitchPos> (get_safety_switch());
}

SwitchPos ET16S::get_r_switch(){
	return static_cast<SwitchPos> (channel[switch_d_num].data);
}

float ET16S::get_wheel(){
	// index will need to be changed to use a different control for wheel
	return channel[l_slider_num].data;
}

void ET16S::send_to_comms() {
	Comms::Sendable<ET16SData> sendable;
	sendable.data = get_ET16S_data();
	sendable.send_to_comms();
}

bool ET16S::is_safety_mode() {
	return (get_safety_switch() == SwitchPos::FORWARD) || !is_connected();
}

bool ET16S::is_teensy_mode() {
	return get_safety_switch() == SwitchPos::MIDDLE;
}

bool ET16S::is_hive_mode() {
	return get_safety_switch() == SwitchPos::BACKWARD;
}

bool ET16S::mode_changed(){
	return mode_changed_flag;
}


ET16SData ET16S::get_ET16S_data(){
	ET16SData ET16S_data;
	ET16S_data.safety_switch = get_safety_switch();
	ET16S_data.r_stick_x = get_r_stick_x();
	ET16S_data.r_stick_y = get_r_stick_y();
	ET16S_data.l_stick_x = get_l_stick_x();
	ET16S_data.l_stick_y = get_l_stick_y();
	ET16S_data.switch_b = get_switch_b();
	ET16S_data.switch_c = get_switch_c();
	ET16S_data.switch_d = get_switch_d();
	ET16S_data.switch_e = get_switch_e();
	ET16S_data.switch_f = get_switch_f();
	ET16S_data.switch_g = get_switch_g();
	ET16S_data.switch_h = get_switch_h();
	ET16S_data.l_slider = get_l_slider();
	ET16S_data.r_slider = get_r_slider();
	ET16S_data.trim_one = get_trim_one();
	ET16S_data.trim_two = get_trim_two();
	ET16S_data.trim_three = get_trim_three();
	ET16S_data.trim_four = get_trim_four();
	ET16S_data.trim_five = get_trim_five();
	ET16S_data.trim_six = get_trim_six();
	ET16S_data.l_dial = get_l_dial();
	ET16S_data.r_dial = get_r_dial();

	return ET16S_data;
}

void ET16S::manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, bool not_safety_mode, float& feed, float& last_feed) {	
	float delta = control_input_timer.delta();
	
	vtm_pos_x += ref.ref_data.kbm_interaction.mouse_speed_x * 0.05 * delta;
	vtm_pos_y += ref.ref_data.kbm_interaction.mouse_speed_y * 0.05 * delta;

	float pitch_min = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.min;
    float pitch_max = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.max;
    float pitch_average = 0.5 * (pitch_min + pitch_max);
    pitch_min -= pitch_average;
    pitch_max -= pitch_average;

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
		chassis_vel_y = -(get_l_stick_x() * 5.4) +
						(ref.ref_data.kbm_interaction.key_d - ref.ref_data.kbm_interaction.key_a) * 2.5;
		
	} else if (estimated_state_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Position) { // if we should be controlling position
		chassis_pos_x = get_l_stick_x() * 2 + pos_offset_x;
		chassis_pos_y = get_l_stick_y() * 2 + pos_offset_y;
	}

	float chassis_spin = get_wheel() * 25;
	float pitch_target = 1.57 + -get_r_stick_y() * 0.3 + vtm_pos_y;
	float yaw_target = -get_r_stick_x() * 1.5 - vtm_pos_x;

	float fly_wheel_target =
		(get_switch_b() == SwitchPos::FORWARD || get_switch_b() == SwitchPos::MIDDLE) ? 18 : 0; // m/s
	// if the right switch is forward, and either the left mouse button is pressed or the right switch is not
	// backward, set the feeder to something. Otherwise, set it to 0
	float feeder_target = ((ref.ref_data.kbm_interaction.button_left || get_switch_h() == SwitchPos::BACKWARD) && fly_wheel_target > 0) ? 12 : 0;
	if (estimated_state_map[Cfg::StateName::Feeder].config().governor_type == Cfg::StateOrder::Position) {
		float dt2 = timer.delta();
		if (dt2 > 0.1) dt2 = 0;
		// check if the shooter is active
		if (not_safety_mode && ref.ref_data.robot_performance.shooter_power_active) {
			feed += feeder_target * dt2;
		}
		
		target_state_map[Cfg::StateName::Feeder].set_position((int)feed);
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
