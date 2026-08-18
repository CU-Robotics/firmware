#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

/// @brief Data struct for testing purposes
struct TestData : Comms::CommsData {
    TestData() : Comms::CommsData(Comms::TypeLabel::TestData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TestData)) {}
    /// @brief x value
    float x = 1.f;
    /// @brief y value
    float y = 2.f;
    /// @brief z value
    float z = 3.f;
    /// @brief w value
    uint32_t w = 0x98765432;
};

/// @brief Data struct for testing purposes
struct BigTestData : Comms::CommsData {
    BigTestData() : Comms::CommsData(Comms::TypeLabel::BigTestData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(BigTestData)) {}

    /// @brief a bunch of data
    float blah[128] = { 0 };
};

/// @brief Struct used to measure the round trip latency of the comms system
struct TestLatencyData : Comms::CommsData {
    TestLatencyData() : Comms::CommsData(Comms::TypeLabel::TestLatencyData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TestLatencyData)) {}
    /// @brief The time in micros when the packet was sent
    uint64_t current_time = 0;
    /// @brief the difference in time between the current time and the time that was contained in the last received packet
    uint64_t time_since_last_received = 0;
};
