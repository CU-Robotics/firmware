#include "logger.hpp"
#include <algorithm>

/// @brief internal buffer with 4kb capacity
/// @note will need to change size if we every print more than 4096 characters per loop
DMAMEM char log_buffer[BUFFER_SIZE];

size_t Logger::write(const uint8_t* buffer, size_t size) {
    // guard against cursor going beyond buffer size
    if (cursor + size >= sizeof(log_buffer)) { return 0; }

    //writes incoming content to cursor position
    memcpy(log_buffer + cursor, buffer, size);
    char print_statement[sizeof(log_buffer)] = { 0 }; // temp variable

    //print_statement captures only the most recent addition to log_buffer
    memcpy(print_statement, log_buffer + cursor, size);

    //precompiler toggle for printing to monitor
#ifdef LOGGER_FLAG
    Serial.print(print_statement);
#endif

    //sets cursor to current place in memory
    cursor += size;
    return size;
}

uint32_t Logger::grab_log_data(uint32_t size, uint8_t* data) {
    // ensure data isn't a null pointer
    if (data == nullptr) { return 0; }

    // make sure we don't copy more than the buffer size
    uint32_t bytes_to_copy = std::min(cursor, size);

    // copy internal buffer into *data
    memcpy(data, log_buffer, bytes_to_copy);

    // reset cursor
    cursor = 0;

    // return number of bytes copied
    return bytes_to_copy;
}
