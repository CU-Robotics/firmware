#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include "IMUSensor.hpp" // abstract parent
// adafruit library specific to LSM6DS(...) hardware
#include <Adafruit_LSM6DSOX.h>

/// @brief Sensor access for an LSM6DSOX IMU Sensor. Child of the abstract IMUSensor class. 
class LSM6DSOX : public IMUSensor {
public:
    /// @brief Constructor. Currently does nothing, use init() instead.
    LSM6DSOX();
    /// @copydoc IMUSensor::read()    
    void read() override;
    /// @copydoc IMUSensor::init()
    void init() override;

private:
    /// @brief sensor object from adafruit libraries.
    Adafruit_LSM6DSOX sensor;
};

#endif