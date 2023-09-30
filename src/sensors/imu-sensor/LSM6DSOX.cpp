#include <LSM6DSOX.hpp>

LSM6DSOX::LSM6DSOX() {
    // initialize LSM

    // start I2C communication 
    sensor.begin_I2C();

    // set data ranges
    sensor.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    sensor.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); // LSM does not support 4000 DPS 

    sensor.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    sensor.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
}

void LSM6DSOX::read() {
    // get the event data from the sensor class
    sensor.getEvent(&accel, &gyro, &temp);

    // assign result to this object's members.
    // (could increase efficiency by specifying which values we need, and only assigning values from that. However, getEvent will read all values from the sensor regardless)
    accel_X = accel.accel.x;
    accel_Y = accel.accel.y;
    accel_Z = accel.accel.z;
    gyro_X = gryo.gyro.x;
    gyro_Y = gryo.gyro.y;
    gyro_Z = gryo.gyro.z;

    temp = temp.temperature;
}