#ifndef ICMSENSOR_H
#define ICMSENSOR_H

#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <Adafruit_Sensor.h>

class IMUSensor {
public:
    virtual void read();
    virtual void init();

    inline float get_temperature() { return temperature; };

    inline float get_accel_X() { return accel_X; };
    inline float get_accel_Y() { return accel_Y; };
    inline float get_accel_Z() { return accel_Z; };

    inline float get_gyro_X() { return gyro_X; };
    inline float get_gyro_Y() { return gyro_Y; };
    inline float get_gyro_Z() { return gyro_Z; };

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