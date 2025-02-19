#ifndef DR16_HPP
#define DR16_HPP

#include <cstdint>		// for access to fixed-width types
#include "Arduino.h"	// for access to HardwareSerial defines

constexpr uint16_t DR16_PACKET_SIZE = 18;	// the size in bytes of a DR16-Receiver packet
constexpr uint16_t DR16_INPUT_VALUE_COUNT = 7;	// the size in floats of the normalized input

constexpr uint16_t DR16_CONTROLLER_INPUT_HIGH = 1684;	// the maximum joystick input value
constexpr uint16_t DR16_CONTROLLER_INPUT_LOW = 364;	// the minimum joystick input value
constexpr uint16_t DR16_CONTROLLER_INPUT_ZERO = 1024;	// the medium joystick input value

constexpr uint16_t DR16_CONTROLLER_SWITCH_HIGH = 3;	// the maximum switch input value
constexpr uint16_t DR16_CONTROLLER_SWITCH_LOW = 1;	// the minimum switch input value

constexpr uint32_t DR16_FAIL_STATE_TIMEOUT = 250000;

/// DR16 Packet Structure
/// (translated from this: https://rm-static.djicdn.com/tem/17348/4.RoboMaster%20%E6%9C%BA%E5%99%A8%E4%BA%BA%E4%B8%93%E7%94%A8%E9%81%A5%E6%8E%A7%E5%99%A8%EF%BC%88%E6%8E%A5%E6%94%B6%E6%9C%BA%EF%BC%89%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)

// Total Size: 18 bytes
// BYTE | BIT layout (format is 'channel # . bit # of channel')
// 1    |	c0.7	c0.6	c0.5	c0.4	c0.3	c0.2	c0.1	c0.0
// 2    |	c1.4	c1.3	c1.2	c1.1	c1.0	c0.10	c0.9	c0.8
// 3    |	c2.1	c2.0	c1.10	c1.9	c1.8	c1.7	c1.6	c1.5
// 4    |	c2.9	c2.8	c2.7	c2.6	c2.5	c2.4	c2.3	c2.2
// 5    |	c3.6	c3.5	c3.4	c3.3	c3.2	c3.1	c3.0	c2.10
// 6    |	s2.1	s2.0	s1.1	s1.0	c3.10	c3.9	c3.8	c3.7
// 7    |							mX	(0-7)
// 8    |							mX 	(8-15)
// 9    |							mY 	(0-7)
// 10   |							mY 	(8-15)
// 11   |							mZ	(0-7)
// 12   |							mZ	(8-15)
// 13   |							m1	(0-7)
// 14   |							m2	(0-7)
// 15   |						KEY 		(0-7)
// 16   |						KEY 		(8-15)
// 17   |						RESERVED (wheel [0-7])
// 18   |						RESERVED (wheel [5-7])

// Channel 	|	Physical Action			|	Value Ranges [low - '0' - high]
// c0		|	Right Joystick X-Axis	|	[364 - 1024 - 1684]
// c1		| 	Right Joystick Y-Axis	| 	[364 - 1024 - 1684]
// c2		| 	Left Joystick X-Axis	|	[364 - 1024 - 1684]
// c3		| 	Left Joystick Y-Axis	|	[364 - 1024 - 1684]
// s1		| 	Left Switch				| 	[1 - 3]
// s2		| 	Right Switch			| 	[1 - 3]
// mX		| 	Mouse X-Axis			|	[-32768 - 0 - 32767]
// mY		| 	Mouse Y-Axis			|	[-32768 - 0 - 32767]
// mZ		| 	Mouse Z-Axis			|	[-32768 - 0 - 32767]
// m1		|	Mouse Left Button		| 	[0 - 1]
// m2		|	Mouse Right Button		| 	[0 - 1]
// key		|	Specific Key Presses	|	Bitmap: (bit #) (0 - 1)
//												W: 0	Q:		4
//												S: 1	E:		5
//												A: 2	Shift:	6
//												D: 3	Ctrl:	7
// wheel	|	Wheel Axis				|	[364 - 1024 - 1684]

/// @brief Wrapper for reading and mapping input from the DR16-Receiver and associated controller
class DR16 {
public:
	/// @brief Constructor, left empty
	DR16();

	/// @brief Initializes DR16 receiver, starts the Serial interface, and zeros input buffers
	void init();

	/// @brief Attempts to read a full packet from the receiver. This function shouldn't be ran more than 100kHz
	void read();

	/// @brief Zeros the normalized input array
	void zero();

public:
	/// @brief Returns the fail bit. Set only if invalid packets have been received for more then 250ms
	/// @return Failure status
	uint8_t is_fail() { return m_fail; }

	/// @brief Returns a the current connection status for the dr16 controller
	/// @return true for connected false for not connected
	bool is_connected() { return m_connected; }

	/// @brief Get the 7 float length input buffer. These values are normalized [-1, 1]
	/// @return float buffer
	float* get_input();

	/// @brief Get right stick x value
	/// @return Right stick x value [-1.f, 1.f]
	float get_r_stick_x();

	/// @brief Get right stick y value
	/// @return Right stick y value [-1.f, 1.f]
	float get_r_stick_y();

	/// @brief Get left stick x value
	/// @return Left stick x value [-1.f, 1.f]
	float get_l_stick_x();

	/// @brief Get left stick y value
	/// @return Left stick y value [-1.f, 1.f]
	float get_l_stick_y();

	/// @brief Get wheel value
	/// @return Wheel value [-1.f, 1.f]
	float get_wheel();

	/// @brief Get left switch value
	/// @return Switch value [1, 2, 3]
	float get_l_switch();

	/// @brief Get right switch value
	/// @return Switch value [1, 2, 3]
	float get_r_switch();

	/// @brief Prints the normalized input buffer
	void print();

	/// @brief Prints the raw 18-byte packet from the receiver
	void print_raw();

	/// @brief Get mouse velocity x
	/// @return Amount of points since last read
	int get_mouse_x();

	/// @brief Get mouse velocity y
	/// @return Amount of points since last read
	int get_mouse_y();

	/// @brief status of left mouse button
	/// @return Is left mouse button pressed
	bool get_l_mouse_button();

	/// @brief status of right mouse button
	/// @return Is right mouse button pressed
	bool get_r_mouse_button();

	/// @brief Get raw 18-byte packet
	/// @return 18-byte packet
	uint8_t* get_raw() { return m_inputRaw; }

private:
	/// @brief Maps the input value to a specified value range
	/// @param value the input value
	/// @param in_low the lowest value the input could be
	/// @param in_high the highest value the input could be
	/// @param out_low the lowest the mapped value should be
	/// @param out_high the highest the mapped value should be
	/// @return Mapped input in the range of [out_low, out_high]
	float bounded_map(int value, int in_low, int in_high, int out_low, int out_high);

	/// @brief A simple check to see if read data is within expected values
	/// @return True/false whether data is deemed valid or not
	bool is_data_valid();

	/// @brief Keep track of mouse x velocity
	int16_t mouse_x;
	/// @brief Keep track of mouse y velocity
	int16_t mouse_y;

	/// @brief Keep track of left mouse button status
	bool l_mouse_button;
	/// @brief Keep track of right mouse button status
	bool r_mouse_button;

public:

	/// @brief keeps track of keys pressed on the rm client
	struct Keys {
		// just testing with keys at the moment
		// but will eventually implement
		// the mouse functionalities.

		/// @brief If the key 'w' is pressed
		bool w;
		/// @brief If the key 's' is pressed
		bool s;
		/// @brief if the key 'a' is pressed
		bool a;
		/// @brief if the key 'd' is pressed
		bool d;
		/// @brief if the key 'shift' is pressed
		bool shift;
		/// @brief if the key 'ctrl' is pressed
		bool ctrl;
		/// @brief if the key 'q' is pressed
		bool q;
		/// @brief if the key 'e' is pressed
		bool e;
		/// @brief if the key 'r' is pressed
		bool r;
		/// @brief if the key 'f' is pressed
		bool f;
		/// @brief if the key 'g' is pressed
		bool g;
		/// @brief if the key 'z' is pressed
		bool z;
		/// @brief if the key 'x' is pressed
		bool x;
		/// @brief if the key 'c' is pressed
		bool c;
		/// @brief if the key 'v' is pressed
		bool v;
		/// @brief if the key 'b' is pressed
		bool b;
	};

	/// @brief struct instance to keep track of the rm control data
	Keys keys;

	/// @brief normalized input buffer
	float m_input[DR16_INPUT_VALUE_COUNT] = { 0 };

	/// @brief raw input split into the 7 input channels
	float m_inputRawSeperated[DR16_INPUT_VALUE_COUNT] = { 0 };

	/// @brief non-normalized, raw 18 byte packet
	uint8_t m_inputRaw[DR16_PACKET_SIZE] = { 0 };

	/// @brief stores previous time value (in micros) for use in calculating a dt
	uint32_t m_prevTime = 0;
	/// @brief time since last valid packet
	uint32_t m_failTime = 0;
	/// @brief fail state
	uint8_t m_fail = false;
	/// @brief connection status
	uint8_t m_connected = false;
	/// @brief keeps track of what time the last packet came in
	uint32_t m_disctTime = 0;
};

#endif // DR16_HPP
