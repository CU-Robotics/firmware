/**
 * @file serial.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines the UART interface for the VectorNav VN100. This primarily involves managing the Teensy serial driver
 *        and providing functions for reading and writing messages.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#pragma once

// FIX: you need to make this entire class use the Teensy hardware serial interface.

#include <Arduino.h>

#include "register_ops.hpp"
#include "bin_parsing.hpp"

namespace vn
{

/**
 * @brief Serial interface for the VectorNav VN100. Provides methods to read and write registers,
 *        as well as manage the UART connection.
 */
class Serial
{
public:
	Serial();
	Serial(HardwareSerial* port);
	~Serial();

	/**
	 * @brief Open the UART connection.
	 *
	 * @return true if the UART was opened successfully, false otherwise.
	 */
	bool open();
	/**
	 * @brief Close the UART connection.
	 *
	 * @return true if the UART was closed successfully, false otherwise.
	 */
	bool close();

	/**
	 * @brief Main loop for the serial interface. Continuously reads data from the UART and processes it.
	 */
	void loop();
	/**
	 * @brief Read a register from the VN100
	 *
	 * @tparam Reg Register type to read
	 * @param reg Reference to the register to read
	 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
	 */
	template<typename Reg>
	ErrorCode read_register(Reg &reg);
	/**
	 * @brief Write a register to the VN100
	 *
	 * @tparam Reg Register type to write
	 * @param reg Reference to the register to write
	 * @note `reg` is filled in with the driver's response packet so you can verify it with the expected response
	 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
	 */
	template<typename Reg>
	ErrorCode write_register(Reg &reg);

	/**
	 * @brief Print the various performance counters
	 */
	void print_status();

public:
	/**
	 * @brief Set the baudrate for the UART connection
	 *
	 * @param baudrate BaudrateSetting enum value to set the baudrate
	 */
	void set_baud(BaudrateSetting baudrate);
	/**
	 * @brief Get the current baudrate setting
	 *
	 * @return BaudrateSetting Current baudrate setting
	 */
	BaudrateSetting get_baud() const;
	/**
	 * @brief Set the UART port to use
	 *
	 * @param serial Pointer to the HardwareSerial object to use
	 */
	void set_port(HardwareSerial* serial);
	/**
	 * @brief Get the current UART port string
	 *
	 * @return HardwareSerial* Pointer to the current HardwareSerial object
	 */
	HardwareSerial *get_port() const;
	/**
	 * @brief Set the callback function for binary messages
	 *
	 * @param callback Callback function to be called when a binary message is received
	 * @note The callback should accept a const ref binary buffer and its length as parameters.
	 */
	void set_binary_callback(bin::binary_callback_t callback);

private:
	/**
	 * @brief Read data from the UART
	 *
	 * @param size Number of bytes to read
	 * @return ssize_t Number of bytes read, or -1 on error
	 */
	ssize_t read(size_t size);
	/**
	 * @brief Write data to the UART
	 *
	 * @param buffer Pointer to the data to write
	 * @param size Number of bytes to write
	 * @return ssize_t Number of bytes written, or -1 on error
	 */
	ssize_t write(const uint8_t *buffer, size_t size);
	/**
	 * @brief Read data from the UART until a response is received for a specific register ID and message type or timeout occurs
	 *
	 * @param reg_id Register ID to wait for
	 * @param msg_type Message type to wait for
	 * @param response_buf Buffer to store the response message
	 * @param response_len Length of the response message after reading
	 * @return ErrorCode ErrorCode::OK on success, any other enum value on failure
	 */
	ErrorCode read_until_response(uint8_t reg_id, msg::MessageType msg_type, msg::buf_ref_t response_buf,
				      msg::len_t &response_len);
	/**
	 * @brief Process the read buffer and extract complete messages
	 * @param bytes_read Number of bytes read from the UART
	 */
	void process_read_buffer(ssize_t bytes_read);
	/**
	 * @brief Process a complete message received from the VN100
	 *
	 * @param msg Pointer to the message data
	 * @param len Length of the message
	 */
	void process_message(const uint8_t *msg, msg::len_t len);
	/**
	 * @brief Set the target message to wait for
	 *
	 * @param reg_id Register ID of the target message
	 * @param msg_type Message type of the target message
	 */
	void set_target_message(uint8_t reg_id, msg::MessageType msg_type);
	/**
	 * @brief Clear the target message state
	 */
	void clear_target_message();

	enum class ProcessingState {
		SEARCH_SOF, 	// searching for the start of a message
		PROCESS_ASCII, 	// processing ASCII messages
		PROCESS_BINARY, // processing binary messages
	};
	/**
	 * @brief Transition to a new processing state
	 *
	 * @param new_state New processing state to transition to
	 */
	void transition_to(ProcessingState new_state);

private:
	HardwareSerial* _uart{nullptr}; // pointer to the UART interface (e.g., Serial1, Serial2, etc.)
	BaudrateSetting _baudrate{BaudrateSetting::BAUD_115200};	// current baudrate setting

	uint8_t _read_buffer[1024] {}; // buffer for reading data from the VN100
	bin::bin_buf_t _curr_msg_buffer{}; // buffer for the current message being processed
	msg::len_t _curr_msg_pos{0}; // length and position of the current message
	bin::BinaryMessage _curr_binary_message{};

	msg::buf_t _outgoing_msg_buf{}; // buffer for outgoing messages
	msg::len_t _outgoing_msg_len{0}; // length of the outgoing message

	msg::buf_t _incoming_msg_buf{}; // buffer for incoming messages
	msg::len_t _incoming_msg_len{0}; // length of the incoming message

	ProcessingState _processing_state{ProcessingState::SEARCH_SOF}; // current processing state

	bin::binary_callback_t _binary_callback{nullptr}; // callback for binary messages

	// target message state
	bool _target_msg_request{false};
	msg::MessageType _target_msg_type{msg::MessageType::Unknown}; // type of the target message
	uint8_t _target_reg_id{255}; // register ID for the target message

	msg::buf_t _target_msg{}; // buffer for the target message
	msg::len_t _target_msg_len{0}; // length of the target message

	uint32_t _read_timeout_us{100'000}; // read timeout in microseconds
	// FIX: is this needed on teensy?
	uint32_t _baudrate_change_delay{10'000}; // delay for baudrate change stabilization

};

} // namespace vn
