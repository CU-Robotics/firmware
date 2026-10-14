#pragma once

#include <string.h>

#include "comms/data/comms_data.hpp"

struct LogData : Comms::CommsData {
    /// @brief Default constructor. Initializes with proper type label, medium, priority, and size.
    LogData() : CommsData(
            Comms::TypeLabel::LogData, 
            Comms::PhysicalMedium::Ethernet, 
            Comms::Priority::Medium, 
            sizeof(LogData)) { }

    /// @brief Initializes with proper type label, medium, priority, and size. Copies provided cstring to data
    /// @param data_cpy cstring to copy as data
    LogData(const char* data_cpy) : CommsData(
            Comms::TypeLabel::LogData, 
            Comms::PhysicalMedium::Ethernet, 
            Comms::Priority::Medium, 
            sizeof(LogData)) { copy_char(data_cpy); }
    
    /// @brief The log text sent over comms
    char data[128];

private:
    /// @brief Copies provided cstring into struct data
    /// @param data_cpy cstring to copy
    void copy_char(const char* data_cpy) {
        strncpy(data, data_cpy, sizeof(data));
    }
};
