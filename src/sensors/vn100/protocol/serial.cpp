/**
 * @file serial.cpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Implements the UART interface for the VectorNav VN100. This primarily involves managing the Teensy serial driver
 *        and providing functions for reading and writing messages.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#include "serial.hpp"

namespace vn {

Serial::Serial() {}

Serial::Serial(HardwareSerial *port)
    : Serial()  // defer to default constructor
{
    set_port(port);
    set_baud(_baudrate);
}

Serial::~Serial() { close(); }

bool Serial::open() {
    if (!_uart) {
        ::Serial.println("No UART port set");
        return false;
    }

    _uart->begin(static_cast<uint32_t>(_baudrate));
    delayMicroseconds(_baudrate_change_delay);

    return true;
}

bool Serial::close() {
    if (!_uart) {
        ::Serial.println("No UART port set");
        return false;
    }

    _uart->end();

    return true;
}

void Serial::loop() {
    // read as much data as possible from the UART
    size_t bytes_read = read(sizeof(_read_buffer));

    if (bytes_read > 0) {
        process_read_buffer(bytes_read);
    }
}

ssize_t Serial::read(size_t size) {
    if (!_uart) {
        ::Serial.println("No UART port set");
        return -1;
    }

    size_t bytes_read = _uart->readBytes(_read_buffer, size);
    _read_buffer[sizeof(_read_buffer) - 1] = '\0';

    if (bytes_read > sizeof(_read_buffer)) {
        ::Serial.println("UART RX BUFFER OVERRUN");
        return -1;
    }

    return bytes_read;
}

ssize_t Serial::write(const uint8_t *buffer, size_t size) {
    if (!_uart) {
        ::Serial.println("No UART port set");
        return -1;
    }

    size_t bytes_written = _uart->write(buffer, size);

    if (bytes_written != size) {
        ::Serial.printf("Failed to write all bytes to UART: %zd/%zu\n", bytes_written, size);
        return -1;
    }

    return bytes_written;
}

void Serial::set_baud(BaudrateSetting baudrate) {
    if (baudrate == _baudrate) {
        return;  // no change
    }
    if (!_uart) {
        return;
    }

    _baudrate = baudrate;

    _uart->begin(static_cast<uint32_t>(_baudrate));
    delayMicroseconds(_baudrate_change_delay);  // wait for the baudrate to stabilize
}

BaudrateSetting Serial::get_baud() const { return _baudrate; }

void Serial::set_port(HardwareSerial *serial) {
    _uart = serial;  // set the UART interface to use
}

HardwareSerial *Serial::get_port() const { return _uart; }

void Serial::set_binary_callback(bin::binary_callback_t callback) { _binary_callback = callback; }

ErrorCode Serial::read_until_response(uint8_t reg_id, msg::MessageType msg_type, msg::buf_ref_t response_buf,
				      msg::len_t &response_len)
{
	set_target_message(reg_id, msg_type);

	uint32_t start_time = micros();

	// read, process, and check for the target message. repeat until we receive the target message or timeout
	do {
		ssize_t bytes_read = read(sizeof(_read_buffer));

		if (bytes_read > 0) {
			process_read_buffer(bytes_read);
		}

		if (_target_msg_len > 0 && _target_msg_len <= (signed)sizeof(_target_msg)) {
			// we received the target message
			memcpy(response_buf, _target_msg, _target_msg_len);
			response_len = _target_msg_len;

			clear_target_message();
			return ErrorCode::OK;
		}

	} while (micros() - start_time < _read_timeout_us); // wait until we receive the target message or timeout

	clear_target_message();

	return ErrorCode::Timeout;
}

void Serial::process_read_buffer(ssize_t bytes_read)
{
	// primary reading logic
	// we maintain a current_message buffer
	// on receiving data, we blindly append it
	// if this value is a end byte, and the current message is valid, we process it

	// we enter the SEARCH_SOF state when we are looking for the start of a message
	// this can happen if we finish processing a message and are ready to receive a new one
	// or if we receive a message that is not valid

	// it is not nominal for us to reach the end of the current message buffer
	// this can happen if the baudrate is mismatched

	for (ssize_t i = 0; i < bytes_read; i++) {
		const char c = _read_buffer[i];

		_curr_msg_buffer[_curr_msg_pos++] = c;

		if (_curr_msg_pos >= (signed)sizeof(_curr_msg_buffer)) {
			::Serial.println("Current message buffer overflow, resetting position. Likely wrong baudrate.");
			_curr_msg_pos = 0; // reset the current message position
			continue; // skip processing this byte
		}

		switch (_processing_state) {
		case ProcessingState::SEARCH_SOF: {
				if (c == vn::msg::START) {
					_curr_msg_buffer[0] = c; // start the buffer with the start byte
					_curr_msg_pos = 1; // reset the current message position to 1
					transition_to(ProcessingState::PROCESS_ASCII); // transition to processing ASCII messages

				} else if (c == vn::bin::START) {
					_curr_msg_buffer[0] = c; // start the buffer with the start byte
					_curr_msg_pos = 1; // reset the current message position to 1
					transition_to(ProcessingState::PROCESS_BINARY); // transition to processing binary messages

				} else {
					_curr_msg_pos = 0; // reset the current message position
					continue; // continue to the next byte
				}

				break;
			}

		case ProcessingState::PROCESS_ASCII: {
				// ascii reading process:
				// 1. get the start byte
				// 2. read until the end byte

				// once all the data is gathered, send it to process_message

				if (c == vn::msg::END) { // if this is the end of a message
					// verify the start byte is the first byte
					// this usually occurs on the first call to read after booting
					// also check that the start is valid
					if (_curr_msg_pos < msg::HEADER_SIZE || strncmp((char *)_curr_msg_buffer, "$VN", 3) != 0) {
						_curr_msg_pos = 0; // reset the current message position
						// we just read the last byte of a message, so it is safe to continue onto the next data
						transition_to(ProcessingState::SEARCH_SOF);
						continue;
					}

					// null terminate the buffer
					_curr_msg_buffer[_curr_msg_pos] = 0;

					process_message(_curr_msg_buffer, _curr_msg_pos);

					// reset the _curr_msg_pos index
					_curr_msg_pos = 0;

					transition_to(ProcessingState::SEARCH_SOF);

					continue;

				} else if (c == vn::msg::START) { // if this is the start of a message
					// reset the counter
					_curr_msg_pos = 0;

					// start the buffer like normal
					_curr_msg_buffer[_curr_msg_pos++] = c;

					// no need to transition states, we are still processing ASCII messages
				}

				break;
			}

		case ProcessingState::PROCESS_BINARY: {
				// binary reading process:
				// 1. get the group byte. this tells us how many bytes to expect for the group types
				// 2. get the group types. this tells us payload size
				// 3. get the payload data and then the 2 byte crc

				// once gathered, validate and send to the callback

				// exactly enough data for the group byte
				if (_curr_msg_pos == 2) {
					ErrorCode err = bin::extract_group(_curr_msg_buffer, _curr_binary_message);

					if (err != ErrorCode::OK) {
						_curr_msg_pos = 0; // reset the current message position
						transition_to(ProcessingState::SEARCH_SOF);
						continue;
					}
				}

				// exactly enough data for the group types
				if (_curr_msg_pos == 2 + (_curr_binary_message.num_groups * 2)) {
					ErrorCode err = bin::extract_group_types(_curr_msg_buffer, _curr_binary_message);

					if (err != ErrorCode::OK) {
						_curr_msg_pos = 0; // reset the current message position
						transition_to(ProcessingState::SEARCH_SOF);
						continue;
					}
				}

				// exactly enough data for the CRC
				if (_curr_msg_pos == 2 + (_curr_binary_message.num_groups * 2) + _curr_binary_message.size + 2) {
					ErrorCode err = bin::verify_crc(_curr_msg_buffer, _curr_binary_message);

					if (err != ErrorCode::OK) {
						_curr_msg_pos = 0; // reset the current message position
						transition_to(ProcessingState::SEARCH_SOF);
						continue;
					}

					if (_binary_callback) {
						_binary_callback(_curr_msg_buffer, _curr_msg_pos, _curr_binary_message);
					}

					_curr_msg_pos = 0; // reset the current message position
					// call the binary message callback if set
					transition_to(ProcessingState::SEARCH_SOF); // transition back to searching for the start of a message
				}

				break;
			}

		default:
			break;
		}
	}
}

void Serial::process_message(const uint8_t *msg, msg::len_t len)
{
	// extract the header
	vn::msg::MessageType command;
	uint8_t reg_id;
	ErrorCode err = msg::extract_header(*reinterpret_cast<const msg::buf_t *>(msg), len, &command, &reg_id);

	if (err != ErrorCode::OK) {
		::Serial.printf("Failed to extract header: %s (%lu)\n", err_to_string(err), static_cast<uint32_t>(err));
		::Serial.printf("Message: %.*s\n", len, msg);
		return;
	}

	// if we have a target message set, check if this is the one we want
	// always check target_msg_type, even if reg_id is not set
	if (_target_msg_request && (reg_id == _target_reg_id || _target_reg_id == 255) && command == _target_msg_type) {
		// copy the message to the target buffer
		memcpy(_target_msg, msg, len);
		_target_msg_len = len;

		// no need to process further. the original target setter will handle the target message
		return;
	}

	// if we have a target message set, and this is an error message
	if (_target_msg_request && command == msg::MessageType::Error) {
		// this is an error message for the target register
		// still copy the message to the target buffer and let the original target setter handle it
		memcpy(_target_msg, msg, len);
		_target_msg_len = len;

		// no need to process further. the original target setter will handle the target message
		return;
	}

	// non-target messages are processed here

	return;
}

void Serial::set_target_message(uint8_t reg_id, msg::MessageType msg_type)
{
	_target_msg_request = true;
	_target_reg_id = reg_id;
	_target_msg_type = msg_type;
	_target_msg_len = 0; // reset the target message length
	memset(_target_msg, 0, sizeof(_target_msg));
}

void Serial::clear_target_message()
{
	_target_msg_request = false;
	_target_msg_type = msg::MessageType::Unknown;
	_target_reg_id = 255;
	_target_msg_len = 0;
}

void Serial::transition_to(ProcessingState new_state)
{
	_processing_state = new_state;
}

template <typename Reg>
ErrorCode Serial::read_register(Reg &reg)
{
	// reading process:
	// 1. create the read register message
	// 2. send it
	// 3. wait until the response, or timeout
	// 4. parse it with the register specific op

	// error out on any error

	ErrorCode err = ErrorCode::OK;

	// create a VNRRG message to read the register
	if ((err = msg::create_read_register(Reg::ID, _outgoing_msg_buf, _outgoing_msg_len)) != ErrorCode::OK) {
		::Serial.printf("Failed to create register read message for register ID: %u (%lu: %s)\n", Reg::ID, static_cast<uint32_t>(err),
			  err_to_string(err));
		return err;
	}

	// write the message to the UART
	if (write(reinterpret_cast<const uint8_t *>(_outgoing_msg_buf), _outgoing_msg_len) != _outgoing_msg_len) {
		::Serial.printf("Failed to write register read message (%lu: %s)\n",
			  static_cast<uint32_t>(ErrorCode::FailedTransmission), err_to_string(ErrorCode::FailedTransmission));
		return ErrorCode::FailedTransmission;
	}

	// blocking read from the UART
	msg::buf_t response_buf{};
	msg::len_t response_len{};

	// we want either the register's custom header or the default write register header
	msg::MessageType header = Reg::HEADER == msg::MessageType::Unknown ? msg::MessageType::ReadRegister : Reg::HEADER;

	if ((err = read_until_response(Reg::ID, header, response_buf, response_len)) != ErrorCode::OK) {
		::Serial.printf("Failed to read response for register ID: %u (%lu: %s)\n", Reg::ID, static_cast<uint32_t>(err),
			  err_to_string(err));
		return err;
	}

	// parse the response
	if ((err = RegisterOps<Reg>::read(response_buf, response_len, reg)) != ErrorCode::OK) {
		::Serial.printf("Failed to parse response for register ID: %u (%lu: %s)\n", Reg::ID, static_cast<uint32_t>(err),
			  err_to_string(err));
		return err;
	}

	return err;
}

template <typename Reg>
ErrorCode Serial::write_register(Reg &reg)
{
	// writing process:
	// 1. create the write register message
	// 2. send it
	// 3. wait until the response, or timeout
	// 4. parse it with the register specific op. the return is then sent back to the argument

	// error out on any error

	ErrorCode err = ErrorCode::OK;

	// create a VNWRG message to write the register
	if ((err = RegisterOps<Reg>::write(reg, _outgoing_msg_buf, _outgoing_msg_len)) != ErrorCode::OK) {
		::Serial.printf("Failed to create register write message for register ID: %u (%lu: %s)\n", Reg::ID, static_cast<uint32_t>(err),
			  err_to_string(err));
		return err;
	}

	// write the message to the UART
	if (write(reinterpret_cast<const uint8_t *>(_outgoing_msg_buf), _outgoing_msg_len) != _outgoing_msg_len) {
		::Serial.printf("Failed to write register read message to UART (%lu: %s)\n",
			  static_cast<uint32_t>(ErrorCode::FailedTransmission), err_to_string(ErrorCode::FailedTransmission));
		return ErrorCode::FailedTransmission;
	}

	// blocking read from the UART
	msg::buf_t response_buf{};
	msg::len_t response_len{};

	// we want either the register's custom header or the default write register header
	msg::MessageType header = Reg::HEADER == msg::MessageType::Unknown ? msg::MessageType::WriteRegister : Reg::HEADER;

	if ((err = read_until_response(Reg::ID, header, response_buf, response_len)) != ErrorCode::OK) {
		::Serial.printf("Failed to read response for msg: %.9s (%lu: %s)\n", _outgoing_msg_buf, static_cast<uint32_t>(err),
			  err_to_string(err));
		return err;
	}

	// parse the response
	if ((err = RegisterOps<Reg>::read(response_buf, response_len, reg)) != ErrorCode::OK) {
		::Serial.printf("Failed to parse response for msg: %.9s (%lu: %s)\n", _outgoing_msg_buf, static_cast<uint32_t>(err),
			  err_to_string(err));
		return err;
	}

	return ErrorCode::OK;
}

}  // namespace vn

#include "serial.tpp" // include the template implementation
