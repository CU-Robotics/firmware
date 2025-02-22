#include "LSM6DSOX.hpp"

LSM6DSOX::LSM6DSOX() {}

// initialize LSM
void LSM6DSOX::init() {
    // start I2C communication (SPI not supported on LSM6DOX+LIS3MDL hardware)
    sensor.begin_I2C();

    // set data ranges
    sensor.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    sensor.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); // LSM does not support 4000 DPS 

    sensor.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    sensor.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
}

bool LSM6DSOX::read() {
    // get the event data from the sensor class
    sensor.getEvent(&accel, &gyro, &temp);

    // assign result to this object's members.
        // could increase efficiency by specifying which values we need, and only assigning values to the object's members from that. 
        // However, getEvent will read all values from the sensor regardless, and assigning these values is very fast

    accel_X = accel.acceleration.x;
    accel_Y = accel.acceleration.y;
    accel_Z = accel.acceleration.z;
    gyro_X = gyro.gyro.x;
    gyro_Y = gyro.gyro.y;
    gyro_Z = gyro.gyro.z;

    temperature = temp.temperature;
    return true;
}