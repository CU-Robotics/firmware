#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cstdint>
#include <cstddef>   
#include "comms/ethernet_packet.hpp"

// **Constants**

constexpr size_t BUFFER_SIZE = Comms::ETHERNET_PACKET_MAX_SIZE - Comms::ETHERNET_PACKET_HEADER_SIZE;      // Fixed buffer size of 4 KB
constexpr size_t REF_DATA_SIZE = 180;     // Size of referee data
constexpr size_t NUM_CAN_MESSAGES = 16;   // Number of CAN messages
constexpr size_t MAX_SENSORS = 16;        // Maximum number of sensors


// **Sensor Types Enumeration**


enum SensorType : uint8_t {
    BUFFENC = 0,
    ICM = 1,
    REVENC = 2,
    TOF = 3,
    LIDAR = 4,

    // Add other sensor types here
};

//some constants for lidar sensors needed for comms as well as the sensor
/// @brief points per D200 data packet
const int D200_POINTS_PER_PACKET = 12;

/// @brief number of timestamp calibration packets. new packet read rate of 333 Hz = 1 packet / 3ms
const int D200_MAX_CALIBRATION_PACKETS = 333;

/// @brief number of packets stored teensy-side
const int D200_NUM_PACKETS_CACHED = 2;


#endif // CONSTANTS_HPP
