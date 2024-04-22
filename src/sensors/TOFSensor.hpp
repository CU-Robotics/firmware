#ifndef _TOF_Sensor_Class_H
#define _TOF_Sensor_Class_H

#include <Arduino.h>
#include <Wire.h>
// Include TOF sensor library
#include <vl53l4cd_class.h>

/// @brief Default I2C bus for the TOF sensor (Wire2 is pins 24 and 25)
constexpr TwoWire* TOF_DEFAULT_I2C_BUS = &Wire2;
/// @brief Default pin to turn off and on the sensor (-1 to disable this feature)
constexpr int TOF_DEFAULT_SHUTOFF_PIN = -1;

/// @brief A time of flight sensor to measure distance in millimeters
class TOFSensor
{
protected:
    /// @brief communicates with the VL53L4CD sensor
    TwoWire* i2c_bus;

    /// @brief Controls reading and writing to VL53L4CD sensor
    VL53L4CD sensor;

    /// @brief Flag to determine whether it is time to read or time to clear the interrupt
    bool should_read = false;

    /// @brief The most recent distance read from the sensor
    uint16_t latest_distance = 0;

public:
    /// @brief Default constructor
    TOFSensor() : i2c_bus(TOF_DEFAULT_I2C_BUS), sensor(TOF_DEFAULT_I2C_BUS, TOF_DEFAULT_SHUTOFF_PIN) {}

    /// @brief constructor, define the external wire within the class and define sensor object.
    /// @param wire_input  input to initialize communication between VL53L4CD sensor and wire
    /// @param pin controls if the senor is on or off
    TOFSensor(TwoWire* wire_input, int pin) {
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
    /// @return distance (mm) from sensor to object at an instant
    /// @note Returns the last read value if there is no new data to read
    uint16_t read()
    {
        // variable to hold the results.
        VL53L4CD_Result_t results;

        // (Mandatory) interrupt to restart measurements.
        // only call this when it is time to clear the interrupt
        if (!should_read)
        {
            sensor.VL53L4CD_ClearInterrupt();

            // swap to the next operation (reading)
            should_read = !should_read;
            return latest_distance;
        }

        // get the results from the sensor, if there is no new data to read, it will automatically send the last read value.
        sensor.VL53L4CD_GetResult(&results);
        latest_distance = results.distance_mm;

        // swap to the next operation (clearing interrupt)
        should_read = !should_read;

        // return the results
        return latest_distance;
    }
};

#endif