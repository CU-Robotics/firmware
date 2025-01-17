#pragma once

#include <Arduino.h>
#include <string>
//http://www.wflysz.com/wflyftp/ET16S/ET16SENV1.00.pdf
constexpr uint16_t ET16S_PACKET_SIZE = 25;
constexpr uint16_t ET16S_INPUT_VALUE_COUNT=16;
constexpr float max_in=1695;
constexpr float min_in=352;
constexpr uint16_t ERROR=0b0000000000001100;

/// @brief organizes the kinds of inputs the transmitter has
enum class input_kind{
	STICK,
	TWO_SWITCH,
	THREE_SWITCH,
	DIAL,
	WHEEL,
	TRIM,
	FLAG,
	INVALID
};

/// @brief stores data and kind of data for the  15 data channels and 1 flag channel
/// for the W-fly Transmitter
struct input_channel{
	/// @brief stores final mapped data
	float data = 0;

	/// @brief stores associated raw data
	uint16_t raw_format=0;

	/// @brief stores associated kind of channel
	input_kind kind = input_kind::INVALID;
};

/// @brief Class for W-Fly transmitter and reciever to gather and map control data
class ET16S {
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
	uint8_t get_safety();
	
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
	
	/// @brief get channel 5 data
	/// @return channel 5 data
	float get_channel_five();
	
	/// @brief get channel 6 data
	/// @return channel 6 data
	float get_channel_six();
	
	/// @brief get channel 7 data
	/// @return channel 7 data
	float get_channel_seven();
	
	/// @brief get channel 8 data
	/// @return channel 8 data
	float get_channel_eight();
	
	/// @brief get channel 9 data
	/// @return channel 9 data
	float get_channel_nine();
	
	/// @brief get channel 10 data
	/// @return channel 10 data
	float get_channel_ten();
	
	/// @brief get channel 11 data
	/// @return channel 11 data
	float get_channel_eleven();
	
	/// @brief get channel 12 data
	/// @return channel 12 data
	float get_channel_twelve();
	
	/// @brief get channel 13 data
	/// @return channel 13 data
	float get_channel_thirteen();
	
	/// @brief get channel 14 data
	/// @return channel 14 data
	float get_channel_fourteen();
	
	/// @brief get channel 15 data
	/// @return channel 15 data
	float get_channel_fifteen();

	/// @brief getter for connection status
	/// @return false if disconnected
	bool get_connection_status();
	

	
private:
	/// @brief prints the entire raw binary data packet exactly as it is recieved
	/// @param m_inputRaw raw bit array
	/// @note can only be placed in read() function for analytics
	void print_raw_bin(uint8_t m_inputRaw[ET16S_PACKET_SIZE]);
	
	/// @brief maps the channel data to a range dependent on the kind of input it is
	/// @param input channel to be mapped
	/// @return mapped value from -1-1 for joysticks,wheels,dials, and 1,2,3 for switches
	float map_raw(input_channel input);
	
	/// @brief Breaks up data packet into correct 11 bit chunks
	/// @param m_inputRaw takes in 8 bit chunks of the total data packet
	void format_raw(uint8_t m_inputRaw[ET16S_PACKET_SIZE]);

	/// @brief assigns the mapped input values to their respective channels
	void set_channel_data();
	
	/// @brief Defines the kind of input (ex: switch,dial,etc...) for each configurable channel
	void set_config();

	/// @brief tests flag byte for disconnect, sets safety to zero if disconnect
	void test_connection();

	/// @brief Array of channels, each holds its respective raw binary data, mapped data, and the kind of input it is (ex: switch,dial,etc...)
	input_channel channel[ET16S_INPUT_VALUE_COUNT];

	/// @brief signifies whether a disconnect flag has been read
	bool is_connected;
};
