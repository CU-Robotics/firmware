#pragma once

#include "comms_data.hpp"

#include <string.h>
#include <string>

namespace Comms {

struct LoggingData : CommsData {
    static constexpr uint16_t max_size = 4096;

    LoggingData()
        : CommsData(TypeLabel::LoggingData, PhysicalMedium::Ethernet, Priority::Logging, sizeof(CommsData)) {}

    void clear() {
        size = sizeof(CommsData);
        memset(log_buffer, 0, sizeof(log_buffer));
    }

    void load_from_buffer(const char* input, uint16_t bytes) {
        clear();
        uint16_t clamped_size = bytes > max_size ? max_size : bytes;
        size = sizeof(CommsData) + clamped_size;
        memcpy(log_buffer, input, clamped_size);
    }

    void deserialize(const char* input, uint16_t bytes) {
        load_from_buffer(input, bytes);
    }

    std::string get_logs() const {
        return std::string(log_buffer, log_buffer + payload_size());
    }

    uint16_t payload_size() const {
        return size > sizeof(CommsData) ? size - sizeof(CommsData) : 0;
    }

    char log_buffer[max_size] = {0};
};

} // namespace Comms
