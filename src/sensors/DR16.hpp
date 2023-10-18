#ifndef DR16_HPP
#define DR16_HPP

#include <cstdint>		// for access to fixed-width types
#include "Arduino.h"	// for access to HardwareSerial defines

constexpr uint16_t DR16_PACKET_SIZE 			= 18;	// the size in bytes of a DR16-Receiver packet
constexpr uint16_t DR16_INPUT_VALUE_COUNT 		= 7;	// the size in floats of the normalized input

constexpr uint16_t DR16_CONTROLLER_INPUT_HIGH 	= 1684;	// the maximum joystick input value
constexpr uint16_t DR16_CONTROLLER_INPUT_LOW 	= 364;	// the minimum joystick input value
constexpr uint16_t DR16_CONTROLLER_INPUT_ZERO 	= 1024;	// the medium joystick input value

constexpr uint16_t DR16_CONTROLLER_SWITCH_HIGH 	= 3;	// the maximum switch input value
constexpr uint16_t DR16_CONTROLLER_SWITCH_LOW 	= 1;	// the minimum switch input value

#define ENABLE_VALUE_CHECK_SAFETY

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
class DR16
{
public:
	/// @brief Constructor, left empty
	DR16();

	/// @brief Initializes DR16 receiver, starts the Serial interface, and zeros input buffers
	void Init();

	/// @brief Attempts to read a full packet from the receiver. This function shouldn't be ran more than 100kHz
	void Read();

  /// @brief Zeros the normalized input array
  void Zero();

public:
	/// @brief Get the 7 float length input buffer. These values are normalized [-1, 1]
	/// @return float buffer
	float* GetInput();

	// Getters for each individual attribute
	float get_r_stick_x();
    
    float get_r_stick_y();

    float get_l_stick_x();

    float get_l_stick_y();

    float get_wheel();

    float get_l_switch();

    float get_r_switch();

	/// @brief Prints the normalized input buffer
	void Print();

	/// @brief Prints the raw 18-byte packet from the receiver
	void PrintRaw();

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
  bool IsDataValid();

private:
	/// @brief normalized input buffer
	float m_input[DR16_INPUT_VALUE_COUNT] = { 0 };

  /// @brief raw input split into the 7 input channels
  float m_inputRawSeperated[DR16_INPUT_VALUE_COUNT] = { 0 };

	/// @brief non-normalized, raw 18 byte packet
	uint8_t m_inputRaw[DR16_PACKET_SIZE] = { 0 };

};

#endif // DR16_HPP