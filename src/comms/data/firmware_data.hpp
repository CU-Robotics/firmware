#pragma once

#if defined(HIVE)

#include "modules/comms/data/comms_data.hpp"        // for CommsData
#include "modules/comms/data/logging_data.hpp"      // for LoggingData
#include "modules/comms/data/data_structs.hpp"      // for shared data structs

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
    /// @brief TempRobotState data
    TempRobotState temp_robot_state;
    /// @brief DR16 data
    DR16Data dr16_data;

    /// @brief Logging data
    Comms::LoggingData logging_data;

    //two buff encodes
    /// @brief yaw_buff_encoder will have id 0
    Comms::BuffEncoderData yaw_buff_encoder;
    /// @brief pitch_buff_encoder will have id 1
    Comms::BuffEncoderData pitch_buff_encoder;

    //three rev encoders
    /// @brief rev_sensor_0
    Comms::RevSensorData rev_sensor_0;
    /// @brief rev_sensor_1
    Comms::RevSensorData rev_sensor_1;
    /// @brief rev_sensor_2
    Comms::RevSensorData rev_sensor_2;

    //one icm
    /// @brief icm_sensor
    Comms::ICMSensorData icm_sensor;

    //one tof
    /// @brief tof_sensor
    Comms::TOFSensorData tof_sensor;

    //two liadars
    /// @brief lidar_sensor_0
    Comms::LidarSensorData lidar_sensor_0;
    /// @brief lidar_sensor_2
    Comms::LidarSensorData lidar_sensor_1;

    //one dr16
    /// @brief dr16_data
    Comms::DR16Data dr16_data;
};


} // namespace Comms

#endif  // HIVE
