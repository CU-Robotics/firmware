
#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <IMUSensor.hpp> // abstract parent
// adafruit library specific to LSM6DS(...) hardware
#include <Adafruit_LSM6DS>

class LSM6DSOX : public IMUSensor {
public:
    LSM6DSOX();
    
    void read() override;
private:
    // sensor object from adafruit libraries.
    Adafruit_LSM6DSOX sensor;
};

#endif