#pragma once

#if defined(HIVE)

#include "modules/comms/data/comms_data.hpp"
#include "modules/comms/data/logging_data.hpp"
#include "modules/comms/data/data_structs.hpp"

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

namespace Comms {

/// @brief Megastruct for receiving data from Firmware, filled on Hive
struct FirmwareData {
    /// @brief Test data
    TestData test_data;
    /// @brief Logging data
    Comms::LoggingData logging_data;
};


} // namespace Comms

#endif  // HIVE
