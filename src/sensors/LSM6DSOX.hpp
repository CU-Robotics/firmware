#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include "IMUSensor.hpp" // abstract parent
// adafruit library specific to LSM6DS(...) hardware
#include <Adafruit_LSM6DSOX.h>

/// @brief Sensor access for an LSM6DSOX IMU Sensor. Child of the abstract IMUSensor class. 
/// @note We currently use the LSM6DSOX+LIS3MDL, which has an accelerometer, gyroscope, and magnetometer. This board only supports I2C (not SPI) communication.
/// @see Adafruit library this class utilizes: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s_o_x.html
class LSM6DSOX : public IMUSensor {
public:
    /// @brief Constructor. Currently does nothing, use @ref init() instead for initialization.
    LSM6DSOX();

    /// @brief Initialize the sensor
    void init();

    /// @copydoc IMUSensor::read()    
    void read() override;

private:
    /// @brief sensor object from adafruit libraries.
    Adafruit_LSM6DSOX sensor;
};

#endif