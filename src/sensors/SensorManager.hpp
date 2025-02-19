#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "Sensor.hpp"
#include "sensor_constants.hpp"
#include "config_layer.hpp"
#include "d200.hpp"
#include "dr16.hpp"
#include "ICM20649.hpp"
#include <SPI.h>
#include "buff_encoder.hpp"
#include "TOFSensor.hpp"
#include "rev_encoder.hpp"
#include "sensors/d200.hpp"
#include <Arduino.h>
#include "RefSystem.hpp"


#define NUM_SENSOR_TYPE 16
#define NUM_IMU_CALIBRATION 50000

/// @class SensorManager
/// @brief Class to manage sensors on the robot
class SensorManager {
public:

    /// @brief Constructor for the SensorManager class
    SensorManager();

/// @brief Destructor for the SensorManager class
    ~SensorManager();

/// @brief Initialize the sensor manager with configuration data
/// @param config_data pointer to configuration data
    void init(const Config* config_data);

    /// @brief Read all sensor data
    void read();

    /// @brief Get the specified buff encoder sensor from the array
    /// @param index index of the sensor object to get
    /// @return pointer to the buff encoder sensor
    BuffEncoder* get_buff_encoder(int index);

    /// @brief Get the specified ICM sensor from the array
    /// @param index index of the sensor object to get
    /// @return pointer to the ICM sensor
    ICM20649* get_icm_sensor(int index);

    /// @brief Get the specified REV sensor from the array
    /// @param index index of the sensor object to get
    /// @return pointer to the REV sensor
    RevEncoder* get_rev_sensor(int index);

    /// @brief Get the specified TOF sensor from the array
    /// @param index index of the sensor object to get
    /// @return pointer to the TOF sensor
    TOFSensor* get_tof_sensor(int index);

    /// @brief Get the specified LiDAR sensor
    /// @param index index of the sensor object to get, 0 or 1
    /// @return pointer to the LiDAR sensor
    D200LD14P* get_lidar_sensor(int index);

    /// @brief Get the number of sensors of the specified type
    /// @param sensor_type the type of sensor
    /// @return number of sensors of that type
    int get_num_sensors(SensorType sensor_type);


    /// @brief Get the referee system
    /// @return pointer to the referee system
    RefSystem* get_ref() {
        return ref;
    }

    /// @brief Calibrate the IMUs
    void calibrate_imus();

    // TODO: Create function that will create a struct to work with the new comms

private:
    /// @brief Number of buff sensors
    int buff_sensor_count;

    /// @brief Number of ICM sensors
    int icm_sensor_count;

    /// @brief Number of Rev sensors
    int rev_sensor_count;

    /// @brief Number of TOF sensors
    int tof_sensor_count;

    /// @brief Number of LiDAR sensors
    int lidar_sensor_count;

    /// @brief Array to store the number of sensors for each sensor type (Indexes for each sensor type are defined in the config.yaml file)
    int num_sensors[NUM_SENSORS];

    /// @brief Array to store robot ICM IMUs
    ICM20649* icm_sensors[NUM_SENSOR_TYPE];

    /// @brief Array to store robot buff encoders
    BuffEncoder* buff_encoders[NUM_SENSOR_TYPE];

    /// @brief Array to store robot rev encoders
    RevEncoder* rev_sensors[NUM_SENSOR_TYPE];

    /// @brief Array to store TOF sensors
    TOFSensor* tof_sensors[NUM_SENSOR_TYPE];

    /// @brief First LiDAR sensor
    D200LD14P* lidar1;

    /// @brief Second LiDAR sensor
    D200LD14P* lidar2;


    /// @brief Array of LiDAR sensor data structs
    LidarSensorData* lidar_sensors_data;

    /// @brief Referee system
    RefSystem* ref;
};

#endif // SENSOR_MANAGER_HPP