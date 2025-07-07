#pragma once

#include "comms_data.hpp"       // for CommsData
#include <string.h>             // for memcpy
#include <string>               // for std::string

namespace Comms {

/// @todo handle logging behavior
struct LoggingData : Comms::CommsData {
    // choose your label, physical medium, and priority for this data struct here.
    /// @brief Primary constructor, initializes CommsData fields
    LoggingData() : CommsData(TypeLabel::LoggingData, PhysicalMedium::Ethernet, Priority::Logging, sizeof(*this)) { };

    /// @brief Extract data from LoggingData's buffer into \p destination.
    /// This function keeps track of how much we have already serialized, so the same content is not serialized again.
    /// @warning Do not call this function unless you mean to stop (duplicate) data from being available on future serializations!
    /// @param destination Starting address for a byte array to copy the log buffer into
    /// @param size How much of the buffer we want to copy into \p destination.
    void serialize(uint8_t* destination, uint16_t size) {
        memcpy(destination, log_buffer + successful_serialized_offset, size);
        successful_serialized_offset += size;
    }

    /// @brief Copy this object's data buffer with content from input, up to \p size
    /// @param input Starting address of input data
    /// @param size How much data to copy from the input
    /// @note Doing this resets the serialization offset, so the next serialize will be from position 0 again.
    void deserialize(char* input, uint16_t size) {
        this->size = size;
        memcpy(log_buffer, input, this->size);
        successful_serialized_offset = 0;
    }

    /// @brief String output of complete LoggingData buffer contents, including a null termination
    /// @return buffer contents parsed as a string.
    /// @note may cut off a single character if the size of the buffer exactly equals the size of the string in order to make room for null termination.
    std::string get_logs() {
        // add null terminator so string knows where to end.
        uint16_t null_terminator_location = size + 1;
        if (size == max_size) { null_terminator_location = size - 1; } // cut off last character if null terminator would exceed size.
        log_buffer[null_terminator_location] = '\0';

        // convert char array into string
        return std::string(log_buffer);
    }

    /// @brief Whether or not this LoggingData has been completely serialized (i.e. fully sent)
    /// @return True when fully serialized, false otherwise.
    bool all_data_has_been_serialized() {
        return successful_serialized_offset >= size;
    }

    /// @brief How much space left is there to serialize
    /// @return difference between size and successful_serialization_offset. 0 When fully serialized.
    uint16_t remaining_size_to_serialize() {
        return size - successful_serialized_offset;
    }

private:
    /// @brief maximum size of the log buffer
    static constexpr uint16_t max_size = 4096;
    /// @brief internal buffer with 4kb capacity
    /// @note will need to change size if we every print more than 4096 characters per loop
    char log_buffer[max_size] = { 0 };

    /// @brief how much of the buffer has been serialized
    uint64_t successful_serialized_offset = 0;
};

} // namespace Comms
