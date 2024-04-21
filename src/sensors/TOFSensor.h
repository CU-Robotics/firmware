#ifndef _TOF_Sensor_Class_H
#define _TOF_Sensor_Class_H

#include <Arduino.h>
#include <Wire.h>
// Include TOF sensor library
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h> 
#include <assert.h>
#include <stdlib.h>

/// @brief This class initializes the tof sensor with the TwoWire object and an a VL53L4CD object, and gets data from the tof sensor
class TOFSensor {
  protected:

    /// @brief communicates with the VL53L4CD sensor
    TwoWire *i2c_bus; 

    /// @brief Controls reading and writing to VL53L4CD sensor
    VL53L4CD *sensor;

  public: 

    /// @brief constructor, define the external wire within the class and define sensor object.
    /// @param wire_input  input to initialize communication between VL53L4CD sensor and wire
    /// @param pin controls if the senor is on or off
    TOFSensor(TwoWire *wire_input, int pin)
    {
      // initalize the wire and the sensor
      i2c_bus = wire_input;

      // dynamically allocated because the library doesnt have a default compiler and we were running into initialization issues
      sensor = new VL53L4CD(wire_input, pin);

    }

  /// @brief function to initialize the sensor, establish connection, begin data collection
  /// @return void
    void init() {

      // initalize the wire 
      i2c_bus->begin();

      // configure the sensor
      sensor->begin();

      // turn off the sensor before initalizing it
      sensor->VL53L4CD_Off();

      // initialize the sensor
      sensor->InitSensor();

      // set the range timing of the sensor, below is best accuracy, highest possible timing budget
      sensor->VL53L4CD_SetRangeTiming(200, 0);

      // start running the sensor, start measuring
      sensor->VL53L4CD_StartRanging();
    }

    /// @brief function to get the distance to the object from the VL53L4CD sensor.
    /// @return distance (mm) from sensor to object at an instant
    uint16_t read() {

      // variable to hold the results.
      VL53L4CD_Result_t results;

      // (Mandatory) interrupt to restart measurements.
      sensor->VL53L4CD_ClearInterrupt();

      // get the results from the sensor, if there is no new data to read, it will automatically send the last read value.
      sensor->VL53L4CD_GetResult(&results);
      
      // return the results
      return results.distance_mm;
    }
};

#endif