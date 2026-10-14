#pragma once

#include <string.h>

#include "comms/data/comms_data.hpp"

struct LogData : Comms::CommsData {
    LogData() : CommsData(
            Comms::TypeLabel::LogData, 
            Comms::PhysicalMedium::Ethernet, 
            Comms::Priority::Medium, 
            sizeof(LogData)) { }

    LogData(const char* data_cpy) : CommsData(
            Comms::TypeLabel::LogData, 
            Comms::PhysicalMedium::Ethernet, 
            Comms::Priority::Medium, 
            sizeof(LogData)) { copy_char(data_cpy); }
    
    char data[128];

private:
    void copy_char(const char* data_cpy) {
        strncpy(data, data_cpy, sizeof(data));
    }
};
