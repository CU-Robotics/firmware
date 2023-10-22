#ifndef ICM20649_H
#define ICM20649_H

// adafruit library specific to ICM20(...) hardware
#include <Adafruit_ICM20649.h> // https://adafruit.github.io/Adafruit_ICM20X/html/class_adafruit___i_c_m20_x.html

#include "IMUSensor.hpp"

/// @brief Sensor access for an ICM20649 IMU Sensor. Child of the abstract IMUSensor class.
class ICM20649 : public IMUSensor {
public:
    /// @brief Constructor. Currently does nothing, use init() instead.
    ICM20649();
    /// @copydoc IMUSensor::read()    
    void read() override;
    /// @copydoc IMUSensor::init()
    void init() override;
private:
    /// @brief sensor object from adafruit libraries.
    Adafruit_ICM20649 sensor;

    /// @brief calculate the approximate acceleration rates in Hz from the divisor.
    /// @return acceleration data rate in Hz
    float get_accel_data_rate();
    /// @brief calculate the approximate gyroscope rates in Hz from the divisor.
    /// @return gyroscope data rate in Hz 
    float get_gyro_data_rate();

    /// @brief approximate acceleration data rate (Hz) calculated from divisor. 
    float accel_rate; 

    /// @brief approximate gyroscope data rate (Hz) calculated from divisor.
    float gyro_rate; 
};

#endif