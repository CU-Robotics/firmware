#pragma once

#include <Arduino.h>
#include <string>
#include <optional>
#include "./Transmitter.hpp"
//http://www.wflysz.com/wflyftp/ET16S/ET16SENV1.00.pdf

//channel[15] is broken

/// @brief The size of an SBUS packet is 25 bits
constexpr uint16_t ET16S_PACKET_SIZE = 25;
/// @brief There is 16 data bytes and 1 flag byte in each packet
constexpr uint16_t ET16S_INPUT_VALUE_COUNT = 17;
/// @brief maximum raw input value for stick,dial,wheel
constexpr float max_in = 1695;
/// @brief minimum raw input value for stick,dial,wheel
constexpr float min_in = 353;	// this is 352 if Port9 is set to w.bus2
/// @brief flag byte displaying disconnect (found through testing)
constexpr uint16_t ERROR = 0b0000000000001100;

/// @brief organizes the kinds of inputs the transmitter has
enum class InputKind {
	INVALID = 0,
	STICK,
	TWO_SWITCH,
	THREE_SWITCH,
	DIAL,
	SLIDER,
	TRIM,
	FLAG
};
/// @brief enum for all possible inputs on transmitter
enum class ChannelId{
	UNMAPPED = 0,
	L_STICK_X,
	L_STICK_Y,
	R_STICK_X,
	R_STICK_Y,
	L_DIAL,
	R_DIAL,
	SWITCH_A,
	SWITCH_B,
	SWITCH_C,
	SWITCH_D,
	SWITCH_E,
	SWITCH_F,
	SWITCH_G,
	SWITCH_H,
	L_SLIDER,
	R_SLIDER,
	TRIM_1,
	TRIM_2,
	TRIM_3,
	TRIM_4,
	TRIM_5,
	TRIM_6,
	FLAG
};
/// @brief three switch possible positions
/// @note for switch on the front plate forward is up

/// @brief stores data and kind of data for the  15 data channels and 1 flag channel
/// for the W-fly Transmitter
struct InputChannel {
	/// @brief stores final mapped data
	float data = 0;

	/// @brief stores associated raw data
	uint16_t raw_format = 0;

	/// @brief stores associated kind of channel
	InputKind kind = InputKind::INVALID;

	/// @brief stores the specific control on the transmitter
	ChannelId id = ChannelId::UNMAPPED;
};

/// @brief Class for W-Fly transmitter and reciever to gather and map control data
class ET16S : public Transmitter {
public:
	/// @brief Constructor, left empty
	ET16S();
	/// @brief Starts Serial Connection to Reciever, Initilizes immutable transmitter channels, calls set_config function.
	void init();
	/// @brief reads raw data from buffer, formats it and sets it to its respective input channel
	void read();
	/// @brief prints integer input value for every channel
	void print_raw();

	/// @brief prints data in binary for a specific channel
	/// @param channel_num channel number from 0-16 inclusive
	void print_format_bin(int channel_num);

	/// @brief prints all mapped channel values
	void print();

	/// @brief get safety values
	/// @return safety value(1-3) 1 is safe
	SwitchPos get_safety_switch();

	/// @brief get right stick x axis value
	/// @return (-1 to 1)
	float get_r_stick_x();

	/// @brief get left stick x axis value
	/// @return (-1 to 1)
	float get_l_stick_x();

	/// @brief get right stick y axis value
	/// @return (-1 to 1)
	float get_r_stick_y();

	/// @brief get left stick y axis value
	/// @return (-1 to 1)
	float get_l_stick_y();
	
	/// @brief get switch b value 
	/// @return switch b value if it exists otherwise return nothing
	std::optional<SwitchPos> get_switch_b();
	
	/// @brief get switch c value 
	/// @return switch c value if it exists otherwise return nothing
	std::optional<SwitchPos> get_switch_c();
	
	/// @brief get switch d value 
	/// @return switch d value if it exists otherwise return nothing
	std::optional<SwitchPos> get_switch_d();
	
	/// @brief get switch e value 
	/// @return switch e value if it exists otherwise return nothing
	std::optional<SwitchPos> get_switch_e();
	
	/// @brief get switch f value 
	/// @return switch f value if it exists otherwise return nothing
	std::optional<SwitchPos> get_switch_f();
	
	/// @brief get switch g value 
	/// @return switch g value if it exists otherwise return nothing
	std::optional<SwitchPos> get_switch_g();
	
	/// @brief get switch h value 
	/// @return switch h value if it exists otherwise return nothing
	std::optional<SwitchPos> get_switch_h();

	/// @brief get left slider value
	/// @return left slider value if it exists otherwise return nothing
	std::optional<float> get_l_slider();

	/// @brief get right slider value
	/// @return right slider value if it exists otherwise return nothing
	std::optional<float> get_r_slider();

	/// @brief get left dial value
	/// @return left dial value if it exists otherwise return nothing
	std::optional<float> get_l_dial();

	/// @brief get right dial value
	/// @return right dial value if it exists otherwise return nothing
	std::optional<float> get_r_dial();

	/// @brief get trim one value
	/// @return trim one value if it exists otherwise return nothing
	std::optional<float> get_trim_one();

	/// @brief get trim two value
	/// @return trim two value if it exists otherwise return nothing
	std::optional<float> get_trim_two();

	/// @brief get trim three value
	/// @return trim three value if it exists otherwise return nothing
	std::optional<float> get_trim_three();

	/// @brief get trim four value
	/// @return trim four value if it exists otherwise return nothing
	std::optional<float> get_trim_four();

	/// @brief get trim five value
	/// @return trim five value if it exists otherwise return nothing
	std::optional<float> get_trim_five();

	/// @brief get trim six value
	/// @return trim six value if it exists otherwise return nothing
	std::optional<float> get_trim_six();
	
	/// @brief get channel data
	/// @param chan_num is the channel number from 0-16
	/// @return channel data
	std::optional<float> get_channel_data(int chan_num);

	/// @brief getter for connection status
	/// @return false if disconnected
	bool is_connected();
	
	/// @brief returns switch A
	/// @note exists for DR16 Backwards Compatability
	/// @return left most front face switch value
	SwitchPos get_l_switch();
	
	/// @brief returns switch D
	/// @note exists for DR16 Backwards Compatability
	/// @return right most front face switch value
	SwitchPos get_r_switch();
	
	/// @brief returns left slider
	/// @note exists for DR16 Backwards Compatability
	/// @return back left spin wheel value
	float get_wheel();

	/// @brief getter for raw data
	/// @return raw data
	uint8_t* get_raw() { return m_inputRaw; }
	
private:
	/// @brief prints the entire raw binary data packet exactly as it is recieved
	/// @param m_inputRaw raw bit array
	/// @note can only be placed in read() function for analytics
	void print_raw_bin(uint8_t m_inputRaw[ET16S_PACKET_SIZE]);

	/// @brief maps the channel data to a range dependent on the kind of input it is
	/// @param input channel to be mapped
	/// @return mapped value from -1-1 for joysticks,wheels,dials, and 1,2,3 for switches
	float map_raw(InputChannel input);
	/// @brief performs mapping calculation for dials,wheels, and joysticks
	/// @param val is the input value
	/// @param min_out minimum output value (-1)
	/// @param max_out maximum ouput value (1)
	/// @return the output value form -1 to 1
	float map_math(float val, float min_out, float max_out);

	/// @brief Breaks up data packet into correct 11 bit chunks
	/// @param m_inputRaw takes in 8 bit chunks of the total data packet
	void format_raw(uint8_t m_inputRaw[ET16S_PACKET_SIZE]);

	/// @brief averages the last 5 samples of a channel
	/// @param sample is the current sample
	/// @param channel_index is the index of the channel
	/// @return the average of the last 5 samples
	float average_channel(float sample, int channel_index);

	/// @brief assigns the mapped input values to their respective channels
	void set_channel_data();

	/// @brief Defines the kind of input (ex: switch,dial,etc...) for each configurable channel
	void set_config();

	/// @brief tests flag byte for disconnect, sets safety to zero if disconnect
	void test_connection();

	/// @brief Array of channels, each holds its respective raw binary data, mapped data, and the kind of input it is (ex: switch,dial,etc...)
	InputChannel channel[ET16S_INPUT_VALUE_COUNT];

	/// @brief signifies whether a disconnect flag has been read
	bool is_connect = false;
	// switch a (safety switch / index 4) is not configurable
	/// @brief r stick x index
	std::optional<int> r_stick_x_num = 0;
	/// @brief right y stick index
	std::optional<int> r_stick_y_num = 1;
	/// @brief left x stick index
	std::optional<int> l_stick_x_num = 3;
	/// @brief left y stick index
	std::optional<int> l_stick_y_num = 2;
	/// @brief stich b index
	std::optional<int> switch_b_num = 5;
	/// @brief switch c index	
	std::optional<int> switch_c_num = 6;
	/// @brief switch d index	
	std::optional<int> switch_d_num = 7;
	/// @brief switch e index	
	std::optional<int> switch_e_num = 8;
	/// @brief switch f index	
	std::optional<int> switch_f_num = 9;
	/// @brief switch g index	
	std::optional<int> switch_g_num = 10;
	/// @brief switch h index	
	std::optional<int> switch_h_num = 11;
	/// @brief right slider index	
	std::optional<int> r_slider_num = 12;
	/// @brief left slider index	
	std::optional<int> l_slider_num = 14;
	/// @brief right dial index	
	std::optional<int> r_dial_num = 13;
	/// @brief left dial index	
	std::optional<int> l_dial_num=15;
	/// @brief trim one index	
	std::optional<int> trim_one_num;
	/// @brief trim two index	
	std::optional<int> trim_two_num;
	/// @brief trim three index	
	std::optional<int> trim_three_num;
	/// @brief trim four index	
	std::optional<int> trim_four_num;
	/// @brief trim five index	
	std::optional<int> trim_five_num;
	/// @brief trim six index	
	std::optional<int> trim_six_num;

	/// @brief the number of samples to average
	const static int AVERAGE_SAMPLE_COUNT = 2;

	/// @brief circular buffer for storing the last few samples of each channel
	float channel_values_circular_buf[ET16S_INPUT_VALUE_COUNT][AVERAGE_SAMPLE_COUNT] = { 0 };
	/// @brief the index of the next sample to be stored in the circular buffer
	int value_indices[ET16S_INPUT_VALUE_COUNT] = { 0 };

	/// @brief raw data packet
	uint8_t m_inputRaw[ET16S_PACKET_SIZE] = { 0 };
};
