#ifndef ICMSENSOR_H
#define ICMSENSOR_H

#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <Adafruit_Sensor.h>

class IMUSensor {
public:
    virtual void read();

    float get_temperature();

    float get_accel_X();
    float get_accel_Y();
    float get_accel_Z();

    float get_gyro_X();
    float get_gyro_Y();
    float get_gyro_Z();

protected:
    // sensor events to read from
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    // acceleration values assigned after read()
    float accel_X = 0;
    float accel_Y = 0;
    float accel_Z = 0;

    // gyroscope values assigned after read()
    float gyro_X = 0;
    float gyro_Y = 0;
    float gyro_Z = 0;

    float temperature = 0;
};

#endif