#pragma once

#include "buff_encoder_data.hpp"

#include "comms/data/logging_data.hpp"              // for LoggingData
#include "comms/data/data_structs.hpp"              // for shared data structs
#include "comms/data/comms_data.hpp"                // for CommsData
#include "sensors/RefSystemPacketDefs.hpp"          // for RefData
#include "comms/data/motor_state_data.hpp"


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
    
    /// @brief TargetState data
    TargetState temp_reference;

    /// @brief Estimated state
    EstimatedState estimated_state;
    
    /// @brief Logging data
    Comms::LoggingData logging_data;
    
    std::vector<BuffEncoderData> buff_encoder_data;
    
    //one icm
    /// @brief icm_sensor
    std::vector<ICMSensorData> icm_sensor_data;
    
    std::vector<MotorStateData> motor_states;
    
    //two liadars
    /// @brief lidar vector
    std::vector<LidarDataPacketSI> lidar_data;

    /// @brief Transmitter data
    TransmitterData transmitter_data;

    /// @brief Config section
    ConfigSection config_section;

    /// @brief Referee data
    RefData ref_data;
};

} // namespace Comms
