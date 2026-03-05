#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include "AdafruitIMUSensor.hpp" // abstract parent
// adafruit library specific to LSM6DS(...) hardware
#include <Adafruit_LSM6DSOX.h>

/// @brief Sensor access for an LSM6DSOX IMU Sensor. Child of the abstract AdafruitIMUSensor class. 
/// @see Adafruit library this class utilizes: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s_o_x.html
class LSM6DSOX : public AdafruitIMUSensor {
public:
    /// @brief Constructor. Currently does nothing, use @ref init() instead for initialization.
    LSM6DSOX(const Cfg::LSMImu& config);

    /// @brief Initialize the sensor
    void init();

    /// @copydoc AdafruitIMUSensor::read()    
    bool read() override;

private:
    const Cfg::LSMImu& config;

    /// @brief sensor object from adafruit libraries.
    Adafruit_LSM6DSOX sensor;

};

#endif