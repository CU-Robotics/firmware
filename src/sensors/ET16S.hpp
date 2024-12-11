#pragma once

#include <Arduino.h>
#include <string>
//http://www.wflysz.com/wflyftp/ET16S/ET16SENV1.00.pdf
constexpr uint16_t ET16S_PACKET_SIZE = 25; //Temp Value
constexpr uint16_t ET16S_INPUT_VALUE_COUNT=15; //Temp Value
constexpr float max_in=1695;
constexpr float min_in=352;
// Channel 	|	Physical Action			|	Value Ranges [low - '0' - high]
// c0		|	Right Joystick X-Axis	|	[352 - 1024 - 1695]
// c1		| 	Right Joystick Y-Axis	|   [352 - 1024 - 1695]
// c2		| 	Left Joystick X-Axis	|	[11264 - 32768 - 54240]
// c3		| 	Left Joystick Y-Axis	|	[11264 - 32768 - 54240]
// c4		| 	Trim One				| 	[11264 - 32768 - 54240]
// c5		| 	Trim Two     			| 	[11264 - 32768 - 54240]
// c6		| 	Trim Three				| 	[11264 - 32768 - 54240]
// c7		| 	Trim Four     			| 	[11264 - 32768 - 54240]
// c8		| 	Trim Five				| 	[11264 - 32768 - 54240]
// c9		| 	Trim Six     			| 	[11264 - 32768 - 54240]

enum input_kind{
	STICK,
	TWO_SWITCH,
	THREE_SWITCH,
	DIAL,
	WHEEL,
	TRIM,
	FLAG,
	INVALID
};
struct input_channel{
	float data = 0;
	uint16_t raw_format=0;
	input_kind kind = INVALID;
};

class ET16S {
public:
	/// @brief Constructor, left empty
	ET16S();
	/// @brief Zeros input buffers
	void init();
	void read();

	void print_raw();
	void print_format_bin();
	void print();
	/// @brief gets normalized input buffer
	/// @return float buffer
	//float* get_input(){return channel.data;}
	/// @brief get right stick x value
	/// @return float right stick x value
	float get_r_stick_x();
	float get_l_stick_x();
	float get_r_stick_y();
	float get_l_stick_y();
	uint8_t get_trim_one();
	uint8_t get_trim_two();
	uint8_t get_trim_three();
	uint8_t get_trim_four();
	uint8_t get_trim_five();
	uint8_t get_trim_six();
	uint8_t get_switch_A();
	uint8_t get_switch_B();
	uint8_t get_switch_C();
	uint8_t get_switch_D();
	uint8_t get_swtich_E();
	uint8_t get_switch_F();
	uint8_t get_switch_G();
	uint8_t get_switch_H();
	uint8_t get_slide_L();
	uint8_t get_slide_R();
	uint16_t* get_raw() {return m_inputRaw_format;}
	

	
private:
	void print_raw_bin(uint8_t[ET16S_PACKET_SIZE]);
	float map_raw(input_channel);
	void format_raw(uint8_t[ET16S_PACKET_SIZE]);
	input_kind set_channel_kind(input_kind);
	void set_channel_data();
	void set_config();
	
	uint16_t m_inputRaw_format[ET16S_INPUT_VALUE_COUNT] = {0};
	input_channel channel[ET16S_INPUT_VALUE_COUNT];
	
};
