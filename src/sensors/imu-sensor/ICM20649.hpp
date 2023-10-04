
#ifndef ICM20649_H
#define ICM20649_H

// adafruit library specific to ICM20(...) hardware
#include <Adafruit_ICM20649.h> // https://adafruit.github.io/Adafruit_ICM20X/html/class_adafruit___i_c_m20_x.html

#include "IMUSensor.hpp"

class ICM20649 : public IMUSensor {
public:
    ICM20649();
    void read() override;
private:
    // sensor object from adafruit libraries.
    Adafruit_ICM20649 sensor;

    // calculate the approximate acceleration  rates in Hz from the divisor.
    float get_accel_data_rate();
    // calculate the approximate gyroscope rates in Hz from the divisor.
    float get_gyro_data_rate();

    float accel_rate; // approximate acceleration data rate (Hz) calculated from divisor. 

    float gyro_rate; // approximate gyroscope data rate (Hz) calculated from divisor.
};

#endif