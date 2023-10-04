
#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include "IMUSensor.hpp" // abstract parent
// adafruit library specific to LSM6DS(...) hardware
#include <Adafruit_LSM6DSOX.h>

class LSM6DSOX : public IMUSensor {
public:
    LSM6DSOX();
    
    void read() override;
    void init() override;
private:
    // sensor object from adafruit libraries.
    Adafruit_LSM6DSOX sensor;
};

#endif