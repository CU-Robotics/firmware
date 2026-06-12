#pragma once

#include "buff_encoder_data.hpp"
#include "icm_sensor_data.hpp"
#include "lsm_sensor_data.hpp"
#include "limit_switch_data.hpp"
#include "rev_encoder_data.hpp"
#include "stereo_cam_trigger_data.hpp"
#include "lidar_data_packet_si.hpp"

#include "comms/data/data_structs.hpp"              // for shared data structs
#include "comms/data/comms_data.hpp"                // for CommsData
#include "comms/data/comms_ref_data.hpp"            // for CommsRefData
#include "comms/data/motor_state_data.hpp"
#include "comms/data/configuration_status_data.hpp" // for ConfigurationStatusData


#include <map>                                   // for std::map

namespace Comms {

/// @brief Megastruct for receiving data from Firmware, filled on Hive
struct FirmwareData {
    /// @brief Set a data section in the mega struct.
    /// @param data The data to be set.
    void set_data(CommsData* data);
        
    /// @brief Test data
    TestData test_data;
    /// @brief Big test data
    BigTestData big_test_data;
    /// @brief Test Latency Data
    TestLatencyData latency_data;
    
    /// @brief TargetState data
    TargetState temp_reference;

    /// @brief Estimated state
    EstimatedState estimated_state;
    /// @brief Map of sensor name to buff encoder data
    std::map<Cfg::SensorName, BuffEncoderData> buff_encoder_data_map;
    /// @brief Map of sensor name to lsm imu data
    std::map<Cfg::SensorName, LsmSensorData> lsm_sensor_data_map;
    /// @brief Map of sensor name to rev encoder data
    std::map<Cfg::SensorName, RevSensorData> rev_sensor_data_map;
    /// @brief Map of sensor name to limit switch data
    std::map<Cfg::SensorName, LimitSwitchData> limit_switch_data_map;

    /// @brief Map of sensor name to stereo camera trigger data
    std::map<Cfg::SensorName, StereoCamTriggerData> stereo_camera_trigger_data_map;
    /// @brief Map of sensor name to lidar data
    std::map<Cfg::SensorName, LidarDataPacketSI> lidar_data_map;

    /// @brief Map of sensor name to icm imu data
    std::map<Cfg::SensorName, ICMSensorData> icm_sensor_data_map;
    /// @brief Map of motor name to motor state data
    std::map<Cfg::MotorName, MotorStateData> motor_state_data_map;
    /// @brief Configuration status data. This is sent from firmware to indicate the status of the configuration process.
    ConfigurationStatusData config_status_data;

    /// @brief dr16 Transmitter data
    DR16Data dr16_data;

    /// @brief ET16S Transmitter data
    ET16SData et16s_data;

    /// @brief Referee data
    CommsRefData ref_data;
};

} // namespace Comms
