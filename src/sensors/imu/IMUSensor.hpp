#ifndef ICMSENSOR_H
#define ICMSENSOR_H

#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <Adafruit_Sensor.h>

/// @brief Abstract parent class for all IMUSensors, which give acceleration and gyroscope data. 
class IMUSensor {
public:
    /// @brief read values from the sensor. Call this to update sensor data before accessing them from the getters. 
    virtual void read();

    /// @brief Get the temperature of the sensor
    /// @return temperature in Celcius
    inline float get_temperature() { return temperature; };

    /// @brief Get the acceleration of the sensor in its local x axis.
    /// @return acceleration m/s^2
    inline float get_accel_X() { return accel_X; };

    /// @brief Get the acceleration of the sensor in its local y axis
    /// @return acceleration m/s^2
    inline float get_accel_Y() { return accel_Y; };

    /// @brief Get the acceleration of the sensor in its local z axis
    /// @return acceleration m/s^2
    inline float get_accel_Z() { return accel_Z; };

    /// @brief Get the change in gyroscope orientation relative to the x axis
    /// @return gryoscope x in radians/s
    inline float get_gyro_X() { return gyro_X; };

    /// @brief Get the change in gyroscope orientation relative to the y axis
    /// @return gyroscope y in radians/s
    inline float get_gyro_Y() { return gyro_Y; };

    /// @brief Get the change in gyroscope orientation relative to the z axis
    /// @return gyroscope z in radians/s
    inline float get_gyro_Z() { return gyro_Z; };

    /// @brief Print out all IMU data to Serial for debugging purposes
    void print();

protected:
    // sensor events to read from

    /// @brief acceleration sensor event from adafruit. Read from this to get acceleration data
    sensors_event_t accel;

    /// @brief gyroscope sensor event from adafruit. Read from this to get gyroscope data
    sensors_event_t gyro;

    /// @brief temperature sensor event from adafruit. Read from this to get temperature data
    sensors_event_t temp;

    // acceleration values assignmed after read() 
    /// @brief acceleration x value 
    float accel_X = 0;
    /// @brief acceleration y value
    float accel_Y = 0;
    /// @brief acceleration z value
    float accel_Z = 0;

    // gyroscope values assigned after read()

    /// @brief gyroscope x value
    float gyro_X = 0;
    /// @brief gyroscope y value
    float gyro_Y = 0;
    /// @brief gyroscope z value
    float gyro_Z = 0;

    /// @brief temperature value
    float temperature = 0;
};

#endif