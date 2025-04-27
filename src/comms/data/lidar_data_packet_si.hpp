#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

/// @brief data for a singular LiDAR packet (SI units)
struct LidarDataPacketSI : Comms::CommsData {
    LidarDataPacketSI() : CommsData(Comms::TypeLabel::LidarDataPacketSI, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(LidarDataPacketSI)) { }

    /// @brief number of points per packet
    static constexpr uint32_t D200_POINTS_PER_PACKET = 12;

    /// @brief the id of the lidar module
    uint8_t id = 0;

    /// @brief speed of lidar module (rad/s)
    float lidar_speed = 0;
  
    /// @brief start angle of measurements (rad)
    float start_angle = 0;
    
    // I made these separate arrays because it reduced the packet size due to alignment issues
    // goes from 96 bytes for these two arrays to 60
    /// @brief distances (m)
    float distances[D200_POINTS_PER_PACKET] = {0};

    /// @brief intensity of measurement. units are ambiguous (not documented), but in general "the higher the intensity, the larger the signal strength value"
    uint8_t intensities[D200_POINTS_PER_PACKET] = {0};
    
    /// @brief end angle of measurements (rad)
    float end_angle = 0;
  
    /// @brief timestamp of measurements, from the lidar, calibrated (s)
    float timestamp = 0;

    /// @brief teensy time of when the packet was received, (s)
    float sample_time = 0;

    /// @brief the angle difference between each lidar point (rad)
    /// @return the angle difference between each lidar point (rad)
    float angle_diff() const { return (end_angle - start_angle) / D200_POINTS_PER_PACKET; }

    /// @brief the time it takes for the lidar to sweep from the start to end angles (s)
    /// @return the time it takes for the lidar to sweep from the start to end angles (s)
    float sweep_time() const { return (end_angle - start_angle) / lidar_speed; }

    /// @brief the yaw of the robot when the packet was received (rad)
    float yaw = 0;

    /// @brief the yaw velocity of the robot when the packet was received (rad/s)
    float yaw_velocity = 0;
};
