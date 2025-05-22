#pragma once

#if defined(HIVE)
#include "modules/comms/data/logging_data.hpp"      // for LoggingData
#include "modules/comms/data/data_structs.hpp"      // for shared data structs
#include "modules/comms/data/comms_data.hpp"        // for CommsData
#include "modules/comms/RefSystemPacketDefs.hpp"    // for RefData
#elif defined(FIRMWARE)
#include "comms/data/logging_data.hpp"              // for LoggingData
#include "comms/data/data_structs.hpp"              // for shared data structs
#include "comms/data/comms_data.hpp"                // for CommsData
#include "sensors/RefSystemPacketDefs.hpp"          // for RefData
#endif

#include <vector>                                   // for std::vector

namespace Comms {

/// @brief Megastruct for receiving data from Firmware, filled on Hive
struct FirmwareData {
    /// @brief Set a data section in the mega struct.
    /// @param data The data to be set.
    /// @warning This is not thread safe, call this on local copies only
    void set_data(CommsData* data);
        
    /// @brief Test data
    TestData test_data;
    /// @brief Big test data
    BigTestData big_test_data;
    /// @brief TempRobotState data
    TempRobotState temp_robot_state;

    /// @brief Estimated state
    EstimatedState estimated_state;
    
    /// @brief Logging data
    Comms::LoggingData logging_data;
    
    //two buff encoders
    /// @brief yaw_buff_encoder will have id 0
    BuffEncoderData yaw_buff_encoder;
    /// @brief pitch_buff_encoder will have id 1
    BuffEncoderData pitch_buff_encoder;
    
    //three rev encoders
    /// @brief rev_sensor_0
    RevSensorData rev_sensor_0;
    /// @brief rev_sensor_1
    RevSensorData rev_sensor_1;
    /// @brief rev_sensor_2
    RevSensorData rev_sensor_2;
    
    //one icm
    /// @brief icm_sensor
    ICMSensorData icm_sensor;
    
    //one tof
    /// @brief tof_sensor
    TOFSensorData tof_sensor;
    
    //two liadars
    /// @brief lidar vector
    std::vector<LidarDataPacketSI> lidars[2];

    /// @brief DR16 data
    DR16Data dr16_data;

    /// @brief Config section
    ConfigSection config_section;

    /// @brief Referee data
    RefData ref_data;
};

} // namespace Comms
