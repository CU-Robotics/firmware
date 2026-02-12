#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "Sensor.hpp"
#include "config_layer.hpp"
#include "d200.hpp"
#include "ICM20649.hpp"
#include <SPI.h>
#include "buff_encoder.hpp"
#include "TOFSensor.hpp"
#include "rev_encoder.hpp"
#include "sensors/d200.hpp"
#include <Arduino.h>
#include "RefSystem.hpp"
#include "comms/data/sendable.hpp"
#include "sensors/limit_switch.hpp"


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
    void init();

    void configure(const NewConfig::RobotConfig& config_data);

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
    // TOFSensor* get_tof_sensor(int index);

    /// @brief Get the specified LiDAR sensor
    /// @param index index of the sensor object to get, 0 or 1
    /// @return pointer to the LiDAR sensor
    D200LD14P* get_lidar_sensor(int index);

    /// @brief Get the specified limit switch
    /// @param index index of the sensor object to get
    /// @return pointer to the limit switch
    LimitSwitch* get_limit_switch(int index);

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

    /// @brief Send sensor data to comms
    void send_sensor_data_to_comms();

    /// @brief Set the estimated state of the robot
    /// @param estimated_state array of the estimated state
    void set_estimated_state(float estimated_state[STATE_LEN][3]) {
        memcpy(this->estimated_state, estimated_state, sizeof(this->estimated_state));
    }

private:
    /// @brief Array to store the estimated state of the robot, used by sensors that adjust from the estimated state
    float estimated_state[STATE_LEN][3] = { {0} };

    std::vector<BuffEncoder> buff_encoders;
    std::vector<Comms::Sendable<BuffEncoderData>> buff_encoder_sendables;

    std::vector<RevEncoder> rev_encoders;
    std::vector<Comms::Sendable<RevSensorData>> rev_encoder_sendables;

    std::vector<ICM20649> icm_sensors;
    std::vector<Comms::Sendable<ICMSensorData>> icm_sensor_sendables;

    std::vector<D200LD14P> lidars;
    std::vector<Comms::Sendable<LidarDataPacketSI>> lidar_sensor_sendables;

    /// @brief Referee system
    RefSystem* ref;
};

#endif // SENSOR_MANAGER_HPP