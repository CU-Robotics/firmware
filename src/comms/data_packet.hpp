#ifndef DATA_PACKET_HPP
#define DATA_PACKET_HPP

#include <cstdint>
#include <state.hpp>
#include <vector>
#include <rm_can.hpp>
#include "Sensor.hpp"
#include "sensor_constants.hpp"
#include "estimator_manager.hpp"
#include "config_layer.hpp"
#include "d200.hpp"

/// @brief Structure to hold referee data.
struct RefereeData {
    /// Raw data array for the referee data packet.
    uint8_t ref_data_raw[180] = {0};
};

/// @brief Structure for the buff encoder sensor.
struct BuffEncoderData {
    /// Sensor ID.
    uint8_t id;
    /// Measured angle.
    float m_angle;

    /// @brief Function to print the sensor data.
    void print();
};

/// @brief Structure for the ICM sensor.
struct ICMSensorData {
    /// Sensor ID.
    uint8_t id;
    /// Acceleration in X-axis.
    float accel_X;
    /// Acceleration in Y-axis.
    float accel_Y;
    /// Acceleration in Z-axis.
    float accel_Z;
    /// Gyroscope reading in X-axis.
    float gyro_X;
    /// Gyroscope reading in Y-axis.
    float gyro_Y;
    /// Gyroscope reading in Z-axis.
    float gyro_Z;
    /// Temperature reading.
    float temperature;

    /// @brief Function to deserialize data from a byte array.
    /// @param data Data to deserialize.
    /// @param offset Offset to update as data is read.
    void deserialize(const uint8_t* data, size_t& offset);

    /// @brief Function to print the sensor data.
    void print();
};

/// @brief Structure for the Rev encoder sensor.
struct RevSensorData {
    /// Sensor ID.
    uint8_t id;
    /// Encoder ticks.
    int ticks;
    /// Angle in radians.
    float radians;

    /// @brief Function to deserialize data from a byte array.
    /// @param data Data to deserialize.
    /// @param offset Offset to update as data is read.
    void deserialize(const uint8_t* data, size_t& offset);

    /// @brief Function to print the sensor data.
    void print();
};

/// @brief Structure for the TOF (Time-of-Flight) sensor.
struct TOFSensorData {
    /// Sensor ID.
    uint8_t id;
    /// Latest distance measurement.
    uint16_t latest_distance;

    /// @brief Function to deserialize the TOF sensor data.
    /// @param data Data to deserialize.
    /// @param offset Offset to update as data is read.
    void deserialize(const uint8_t* data, size_t& offset);

    /// @brief Function to print the sensor data.
    void print();
};

/// @brief Structure for the LiDAR sensor.
struct LidarSensorData {
    /// Sensor ID.
    uint8_t id;
    /// Index of the current data packet.
    int current_packet;
    /// Calibration data.
    D200Calibration cal;
    /// Array of cached data packets.
    LidarDataPacketSI packets[D200_NUM_PACKETS_CACHED];

    /// @brief Function to deserialize data from a byte array.
    /// @param data Data to deserialize.
    /// @param offset Offset to update as data is read.
    void deserialize(const uint8_t* data, size_t& offset);

    /// @brief Function to print the latest LiDAR data packet.
    void print_latest_packet();
};

/// @brief Structure for the data packet containing all sensor data and state information.
struct data_packet {
    /// Pointer to the Config struct storing all configuration data.
    const Config* config;

    /// Current time in microseconds.
    uint32_t timestamp;
    /// Robot state.
    State state;

    /// Referee data.
    RefereeData refData;
    /// CAN bus data.
    CANData canData;

    /// Number of buff sensors.
    int buff_sensor_count;
    /// Number of ICM sensors.
    int icm_sensor_count;
    /// Number of Rev sensors.
    int rev_sensor_count;
    /// Number of TOF sensors.
    int tof_sensor_count;
    /// Number of LiDAR sensors.
    int lidar_sensor_count;

    /// Array of buff sensors.
    BuffEncoderData* buff_sensors;
    /// Array of ICM sensors.
    ICMSensorData* icm_sensors;
    /// Array of Rev sensors.
    RevSensorData* rev_sensors;
    /// Array of TOF sensors.
    TOFSensorData* tof_sensors;
    /// Array of LiDAR sensors.
    LidarSensorData* lidar_sensors;

    /// @brief Getter for RefereeData.
    /// @return RefereeData object.
    RefereeData getRefData() const;

    /// @brief Getter for CANData.
    /// @return CANData object.
    CANData getCanData() const;

    /// @brief Constructor to initialize the data_packet with configuration data.
    /// @param config_data Pointer to the configuration data.
    data_packet(const Config* config_data);

    /// @brief Destructor to clean up allocated memory.
    ~data_packet();

    /// @brief Function to pack data into a packet buffer.
    /// @param packetBuffer Buffer to pack the data into.
    /// @param robotState The current state of the robot.
    /// @param ref_data_raw Raw referee data.
    /// @param canData Pointer to CANData.
    /// @param estimatorManager Reference to the EstimatorManager.
    /// @param lidar1 Reference to the first LiDAR sensor.
    /// @param lidar2 Reference to the second LiDAR sensor.
    void pack_data_packet(
        uint8_t packetBuffer[BUFFER_SIZE],
        State robotState,
        uint8_t ref_data_raw[180],
        CANData* canData,
        EstimatorManager& estimatorManager,
        D200LD14P& lidar1,
        D200LD14P& lidar2
    );

    /// @brief Function to unpack data from a packet buffer.
    /// @param packetBuffer Buffer containing the packed data.
    void unpack_data_packet(uint8_t packetBuffer[BUFFER_SIZE]);
};

#endif // DATA_PACKET_HPP