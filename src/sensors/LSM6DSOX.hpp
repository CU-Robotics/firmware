#pragma once

#include "AdafruitIMUSensor.hpp" // abstract parent
// adafruit library specific to LSM6DS(...) hardware
#include <Adafruit_LSM6DSOX.h>
#include "comms/data/lsm_sensor_data.hpp"

/// @brief Sensor access for an LSM6DSOX IMU Sensor. Child of the abstract AdafruitIMUSensor class. 
/// @see Adafruit library this class utilizes: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s_o_x.html
class LSM6DSOX : public AdafruitIMUSensor<LSM6DSOX> {
public:
    /// @brief Default constructor, initiaizes config data and comms_data with name from config.
    /// @param config configuration struct for this LSM6DSOX sensor
    LSM6DSOX(const Cfg::LsmImu& config) : config(config), comms_data(config.imu_name) {}

    /// @brief Initialize the sensor
    void init_impl();

    /// @copydoc AdafruitIMUSensor::read()    
    void read_impl();
    /// @brief Send the sensor data to comms
    void send_to_comms_impl() const;
    /// @brief set the gyro rate range of the sensor
    /// @param range The range to set for the gyroscope
    void set_gyro_range(Cfg::LsmImuGyroRange range);
    /// @brief set the accelerometer range of the sensor
    /// @param range The range to set for the accelerometer
    void set_accel_range(Cfg::LsmImuAccelRange range);

private:
    /// @brief Configuration data for this sensor
    const Cfg::LsmImu& config;
    /// @brief LSM sensor data to be sent to comms
    LsmSensorData comms_data;

    /// @brief sensor object from adafruit libraries.
    Adafruit_LSM6DSOX sensor;

};
