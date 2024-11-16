#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cstdint>
#include <cstddef>   

// **Constants**

constexpr size_t BUFFER_SIZE = 4096;      // Fixed buffer size of 4 KB
constexpr size_t REF_DATA_SIZE = 180;     // Size of referee data
constexpr size_t NUM_CAN_MESSAGES = 16;   // Number of CAN messages
constexpr size_t MAX_SENSORS = 16;        // Maximum number of sensors


// **Sensor Types Enumeration**


enum class SensorType : uint8_t {
    BUFFENC = 0,
    ICM = 1,
    REVENC = 2,
    TOF = 3,
    LIDAR = 4,

    // Add other sensor types here
};

#endif // CONSTANTS_HPP
