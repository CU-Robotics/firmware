#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/
// adafruit library specific to ICM20(...) hardware
#include <Adafruit_ICM20X.h> // https://adafruit.github.io/Adafruit_ICM20X/html/class_adafruit___i_c_m20_x.html

#ifndef ICM20649_H
#define ICM20649_H

class IMU20649{
public:
    IMU20649();
    
    void read();

    float get_temperature();
    
    float get_accel_X();
    float get_accel_Y();
    float get_accel_Z();

    float get_gyro_X();
    float get_gyro_Y();
    float get_gyro_Z();

private:
    // sensor object from adafruit libraries.
    Adafruit_ICM20649 sensor;
    // sensor events to read from
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    // calculate the approximate acceleration  rates in Hz from the divisor.
    float get_accel_data_rate();
    // calculate the approximate gyroscope rates in Hz from the divisor.
    float get_gyro_data_rate();

    
    float accel_rate; // approximate acceleration data rate (Hz) calculated from divisor. 

    // acceleration values assigned after read()
    float accel_X = 0;
    float accel_Y = 0;
    float accel_Z = 0;

    float gyro_rate; // approximate gyroscope data rate (Hz) calculated from divisor.
    // gyroscope values assigned after read()
    float gyro_X = 0;
    float gyro_Y = 0;
    float gyro_Z = 0;

    float temperature = 0;
};

#endif