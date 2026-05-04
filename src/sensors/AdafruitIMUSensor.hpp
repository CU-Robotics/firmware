#pragma once

#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <Adafruit_Sensor.h>
#include "sensors/sensor.hpp"

/// @brief Abstract parent class for all AdafruitIMUSensors, which give acceleration and gyroscope data.
template <typename Derived>
class AdafruitIMUSensor : public Sensor<Derived> {
public:
    /// @brief Constructor that takes a SensorType and passes it to the Sensor constructor.
    AdafruitIMUSensor() : Sensor<Derived>() {}

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
	void print() {
		// Display the temperature data, measured in Celcius
		Serial.print("\t\tTemperature ");
		Serial.print(get_temperature());
		Serial.println(" deg C");
		// Display the acceleration data, measured in m/s^2)
		Serial.print("\t\tAccel X: ");
		Serial.print(get_accel_X());
		Serial.print(" \tY: ");
		Serial.print(get_accel_Y());
		Serial.print(" \tZ: ");
		Serial.print(get_accel_Z());
		Serial.println(" m/s^2 ");
		// Display gyroscope data, measured in radians/s
		Serial.print("\t\tGyro X: ");
		Serial.print(get_gyro_X());
		Serial.print(" \tY: ");
		Serial.print(get_gyro_Y());
		Serial.print(" \tZ: ");
		Serial.print(get_gyro_Z());
		Serial.println(" radians/s ");
		Serial.println();
	}
	/// @brief Prints a live, formatted dashboard of IMU data
	void print_live_data_impl() {
		Serial.printf("=== LIVE ADAFRUIT SENSOR DATA ===\n");
    
		// Print temperature
		Serial.printf(" Temperature: %5.2f °C\n", get_temperature());
		Serial.println("------------------------------------------------");
    
		// Print Acceleration and Gyro data
		Serial.printf(" Accel (m/s^2) | X: %6.2f | Y: %6.2f | Z: %6.2f\n", 
					  get_accel_X(), get_accel_Y(), get_accel_Z());
                  
		Serial.printf(" Gyro (rad/s)  | X: %6.2f | Y: %6.2f | Z: %6.2f\n", 
					  get_gyro_X(), get_gyro_Y(), get_gyro_Z());
	}

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
