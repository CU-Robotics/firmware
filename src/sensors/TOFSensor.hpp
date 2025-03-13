#ifndef _TOF_Sensor_Class_H
#define _TOF_Sensor_Class_H

#include <Arduino.h>
#include <Wire.h>
// Include TOF sensor library
#include <vl53l4cd_class.h>

#include "Sensor.hpp"

/// @brief Default I2C bus for the TOF sensor (Wire2 is pins 24 and 25)
constexpr TwoWire* TOF_DEFAULT_I2C_BUS = &Wire2;
/// @brief Default pin to turn off and on the sensor (-1 to disable this feature)
constexpr int TOF_DEFAULT_SHUTOFF_PIN = -1;

/// @brief Structure for the TOF (Time-of-Flight) sensor.
struct TOFSensorData {
    /// Sensor ID.
    uint8_t id;
    /// Latest distance measurement.
    uint16_t latest_distance;
};

/// @brief A time of flight sensor to measure distance in millimeters
class TOFSensor : public Sensor {
protected:
    /// @brief communicates with the VL53L4CD sensor
    TwoWire* i2c_bus;

    /// @brief Controls reading and writing to VL53L4CD sensor
    VL53L4CD sensor;

    /// @brief Flag to determine whether it is time to read or time to clear the interrupt
    bool should_read = false;


    /// @brief The most recent distance read from the sensor
    uint16_t latest_distance = 0;

    /// @brief struct storing data from TOF sensor
    TOFSensorData tof_sensor_data;

public:
    /// @brief Default constructor
    TOFSensor() : Sensor(SensorType::TOF), i2c_bus(TOF_DEFAULT_I2C_BUS), sensor(TOF_DEFAULT_I2C_BUS, TOF_DEFAULT_SHUTOFF_PIN) { }

    /// @brief constructor, define the external wire within the class and define sensor object.
    /// @param wire_input  input to initialize communication between VL53L4CD sensor and wire
    /// @param pin controls if the senor is on or off
    TOFSensor(TwoWire* wire_input, int pin) : Sensor(SensorType::TOF) {
        // initalize the wire and the sensor
        i2c_bus = wire_input;

        // dynamically allocated because the library doesnt have a default compiler and we were running into initialization issues
        sensor = VL53L4CD(wire_input, pin);
    }

    /// @brief function to initialize the sensor, establish connection, begin data collection
    void init() {
        // initalize the wire 
        i2c_bus->begin();

        // configure the sensor
        sensor.begin();

        // turn off the sensor before initalizing it
        sensor.VL53L4CD_Off();

        // initialize the sensor
        sensor.InitSensor();

        // set the range timing of the sensor, below is best accuracy, highest possible timing budget
        sensor.VL53L4CD_SetRangeTiming(200, 0);

        // start running the sensor, start measuring
        sensor.VL53L4CD_StartRanging();
    }

    /// @brief function to get the distance to the object from the VL53L4CD sensor.
    /// @note Returns the last read value if there is no new data to read
    /// @return true if successful, false if no data available
    bool read() override {
        // variable to hold the results.
        VL53L4CD_Result_t results;

        // (Mandatory) interrupt to restart measurements.
        // only call this when it is time to clear the interrupt
        if (!should_read) {
            sensor.VL53L4CD_ClearInterrupt();

            // swap to the next operation (reading)
            should_read = !should_read;
            return false;
        }

        // get the results from the sensor, if there is no new data to read, it will automatically send the last read value.
        sensor.VL53L4CD_GetResult(&results);
        latest_distance = results.distance_mm;

        // copy the data to the data struct
        tof_sensor_data.id = id_;
        tof_sensor_data.latest_distance = latest_distance;

        // swap to the next operation (clearing interrupt)
        should_read = !should_read;
        return true;
    }

    /// @brief function to deserialize the TOF sensor data
    void print() {
        Serial.println("TOF Sensor:");
        Serial.printf("\tDistance: %u mm\n", latest_distance);
    }

    /// @brief funtion to return latest distance
    /// @return latest distance
    uint16_t get_latest_distance() {
        return latest_distance;
    }
};

#endif