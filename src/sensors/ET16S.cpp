#include "ET16S.hpp"

ET16S::ET16S() { }


void ET16S::init() {
	Serial8.begin(100000, SERIAL_8E1_RXINV_TXINV);
	Serial8.flush();
	Serial8.clear();
	//configure safety switch
	//InputKind three_switch=THREE_SWITCH;
	channel[4].kind = InputKind::THREE_SWITCH;
	//Turn safety on
	channel[4].data = static_cast<float>(SwitchPos::FORWARD);

	//configure sticks
	//right stick horizontal
	channel[0].kind = InputKind::STICK;
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
	// Raw data stored in array
	uint8_t m_inputRaw[ET16S_PACKET_SIZE] = { 0 };
	// Store 2 packets (50 bytes) in the buffer to prevent incomplete packet reads
	if (Serial8.available() < (2*ET16S_PACKET_SIZE)) {
		return;
	}
	// We read until we find the start byte of the new packet (0x0f)
	// this ensures we read one packet per loop
	while (Serial8.peek() != 0x0f) {
		Serial8.read();
	}
	// Fill raw input array
	for (int i = 0; i < ET16S_PACKET_SIZE;i++) {
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
	//if(channel[5].kind==InputKind::THREE_SWITCH){Serial.print("ITS TRUE");}
	///print_raw_bin(m_inputRaw);
	//print_format_bin(16);
	print();
	//print_raw();
}

void ET16S::print() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT; i++) {
		Serial.print(channel[i].data);
		Serial.print(" ");
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
	uint16_t c[22] = { 0 };

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
	channel[0].id = ChannelId::R_STICK_X;
	channel[1].id = ChannelId::R_STICK_Y;
	channel[2].id = ChannelId::L_STICK_Y;
	channel[3].id = ChannelId::L_STICK_X;
	channel[4].id = ChannelId::SWITCH_A;
	channel[5].id = ChannelId::SWITCH_B;
	channel[6].id = ChannelId::SWITCH_C;
	channel[7].id = ChannelId::SWITCH_D;
	channel[8].id = ChannelId::SWITCH_E;
	channel[9].id = ChannelId::SWITCH_F;
	channel[10].id = ChannelId::SWITCH_G;
	channel[11].id = ChannelId::SWITCH_H;
	channel[12].id = ChannelId::R_SLIDER;
	channel[14].id = ChannelId::L_SLIDER;
	channel[13].id = ChannelId::R_DIAL;
	channel[15].id = ChannelId::UNMAPPED; //channel[15] is non-functional
	channel[16].id = ChannelId::FLAG;

	for (int i=5; i<ET16S_INPUT_VALUE_COUNT;i++){
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

void ET16S::set_channel_data() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT;i++) {
		channel[i].data = map_raw(channel[i]);
	}
}

void ET16S::test_connection() {
	uint16_t flag_byte = channel[16].data;
	if (flag_byte & ERROR) {
		is_connected = false;
	} else {
		is_connected = true;
	}
}

uint8_t ET16S::get_safety_switch() {
	return channel[4].data;
}

float ET16S::get_r_stick_x() {
	return channel[0].data;
}

float ET16S::get_r_stick_y() {
	return channel[1].data;
}

float ET16S::get_l_stick_x() {
	return channel[2].data;
}

float ET16S::get_l_stick_y() {
	return channel[3].data;
}
std::optional<float> ET16S::get_switch_b(){
	if (!switch_b_num.has_value()){ return {}; }
		return channel[switch_b_num.value()].data;
}
std::optional<float> ET16S::get_switch_c(){
	if (!switch_c_num.has_value()){ return {}; }
	return channel[switch_c_num.value()].data;
}
std::optional<float> ET16S::get_switch_d(){
	if (!switch_d_num.has_value()){ return {}; }
	return channel[switch_d_num.value()].data;
}
std::optional<float> ET16S::get_switch_e(){
	if (!switch_e_num.has_value()){ return {}; }
	return channel[switch_e_num.value()].data;
}
std::optional<float> ET16S::get_switch_f(){
	if (!switch_f_num.has_value()){ return {}; }
	return channel[switch_f_num.value()].data;
}
std::optional<float> ET16S::get_switch_g(){
	if (!switch_g_num.has_value()){ return {}; }
	return channel[switch_g_num.value()].data;
}
std::optional<float> ET16S::get_switch_h(){
	if (!switch_h_num.has_value()){ return {}; }
	return channel[switch_h_num.value()].data;
}
std::optional<float> ET16S::get_l_slider(){
	if (!l_slider_num.has_value()){ return {}; }
	return channel[l_slider_num.value()].data;
}
std::optional<float> ET16S::get_r_slider(){
	if (!r_slider_num.has_value()){ return {}; }
	return channel[r_slider_num.value()].data;
}
std::optional<float> ET16S::get_trim_one(){
	if (!trim_one_num.has_value()){ return {}; }
	return channel[trim_one_num.value()].data;
}
std::optional<float> ET16S::get_trim_two(){
	if (!trim_two_num.has_value()){ return {}; }
	return channel[trim_two_num.value()].data;
}
std::optional<float> ET16S::get_trim_three(){
	if (!trim_three_num.has_value()){ return {}; }
	return channel[trim_three_num.value()].data;
}
std::optional<float> ET16S::get_trim_four(){
	if (!trim_four_num.has_value()){ return {}; }
	return channel[trim_four_num.value()].data;
}
std::optional<float> ET16S::get_trim_five(){
	if (!trim_five_num.has_value()){ return {}; }
	return channel[trim_five_num.value()].data;
}
std::optional<float> ET16S::get_trim_six(){
	if (!trim_six_num.has_value()){ return {}; }
	return channel[trim_six_num.value()].data;
}
std::optional<float> ET16S::get_l_dial(){
	if (!l_dial_num.has_value()){ return {}; }
	return channel[l_dial_num.value()].data;
}
std::optional<float> ET16S::get_r_dial(){
	if (!r_dial_num.has_value()){ return {}; }
	return channel[r_dial_num.value()].data;
}

std::optional<float> ET16S::get_channel_data(int chan_num){
	// Will return nothing if an incorrect channel number is given
	if ((chan_num < 0) || (chan_num > 16)){
		return {};
	}
	return channel[chan_num].data;
}

bool ET16S::get_connection_status() {
	return is_connected;
}

