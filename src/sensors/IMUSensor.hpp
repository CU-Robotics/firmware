#ifndef ICMSENSOR_H
#define ICMSENSOR_H

#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <Adafruit_Sensor.h>
#include "Sensor.hpp"

/// @brief Abstract parent class for all IMUSensors, which give acceleration and gyroscope data. 
class IMUSensor : public Sensor {
public:
    /// @brief Constructor that takes a SensorType and passes it to the Sensor constructor.
    IMUSensor() : Sensor(SensorType::ICM) {}


    /// @brief read values from the sensor. Call this to update sensor data before accessing them from the getters.
    /// @return true if successful, false if no data available 
    virtual bool read() = 0;

    /// @brief Get the temperature of the sensor
    /// @return temperature in Celcius
    inline float get_temperature() { return temperature; };


    //POSSIBLE BUG REASON: the accelerations are not corrected for the offset possibly being the reason for the weird values.
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
    inline float get_gyro_X() { return gyro_X - offset_X; };

    /// @brief Get the change in gyroscope orientation relative to the y axis
    /// @return gyroscope y in radians/s
    inline float get_gyro_Y() { return gyro_Y - offset_Y; };

    /// @brief Get the change in gyroscope orientation relative to the z axis
    /// @return gyroscope z in radians/s
    inline float get_gyro_Z() { return gyro_Z - offset_Z; };

    /// @brief Set offsets that we calculate during calibration
    /// @param x offset in x
    /// @param y offset in y
    /// @param z offset in z
    inline void set_offsets(float x, float y, float z) {
        offset_X = x;
        offset_Y = y;
        offset_Z = z;
    }

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
    /// @brief offset x value
    float offset_X = 0;
    /// @brief offset y value
    float offset_Y = 0;
    /// @brief offset z value
    float offset_Z = 0;

    /// @brief temperature value
    float temperature = 0;
};

#endif