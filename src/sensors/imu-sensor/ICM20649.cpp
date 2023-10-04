#include "ICM20649.hpp"

ICM20649::ICM20649() {}

// initialize ICM
void ICM20649::init() {
    // start I2C communication 
    
    if (!sensor.begin_I2C()) {
      Serial.println("Failed to begin i2c");
    }

    // set data ranges
    sensor.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    sensor.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);

    // set accel rate (via divisor setting)
    sensor.setAccelRateDivisor(4095);
    accel_rate = get_accel_data_rate();

    // set gyro rate (via divisor setting)
    sensor.setGyroRateDivisor(255);
    gyro_rate = get_gyro_data_rate();
}

void ICM20649::read() {
    // get the event data from the sensor class
    sensor.getEvent(&accel, &gyro, &temp);

    // assign result to this object's members.
    // (could increase efficiency by specifying which values we need, and only assigning values from that. However, getEvent will read all values from the sensor regardless)
    accel_X = accel.acceleration.x;
    accel_Y = accel.acceleration.y;
    accel_Z = accel.acceleration.z;
    gyro_X = gyro.gyro.x;
    gyro_Y = gyro.gyro.y;
    gyro_Z = gyro.gyro.z;

    temperature = temp.temperature;
}

// calculate the approximate acceleration rate (Hz) from the divisor.
float ICM20649::get_accel_data_rate() {
    // equation from Adafruit ICM20649 example code
    return 1125 / (1.0 + sensor.getAccelRateDivisor());
}

float ICM20649::get_gyro_data_rate() {
    return gyro_rate = 1100 / (1.0 + sensor.getGyroRateDivisor());
}