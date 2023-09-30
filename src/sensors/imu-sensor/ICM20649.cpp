#include <ICM20649.hpp>

ICM20649::ICM20649() {
    // initialize ICM

    // start I2C communication 
    sensor.begin_I2C();

    // set data ranges
    sensor.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    sensor.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);

    // set accel rate (via divisor setting)
    sensor.setAccelRateDivisor(4095);
    accel_rate = get_accel_data_rate();

    // set gyro rate (via divisor setting)
    sensor.setGyroRateDivisor();
    gyro_rate = get_gyro_data_rate();
}

float get_temperature() { return temperature; };

float get_accel_X() { return accel_X; };
float get_accel_Y() { return accel_X; };
float get_accel_Z() { return accel_X; };

float get_gyro_X() { return gyro_X; };
float get_gyro_Y() { return gyro_Y; };
float get_gyro_Z() { return gyro_Z; };

void ICM20649::read() {
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

// calculate the approximate acceleration rate (Hz) from the divisor.
float ICM20649::get_accel_data_rate() {
    // equation from Adafruit ICM20649 example code
    return 1125 / (1.0 + sensor.getAccelRateDivisor());
}

float ICM20649::get_gyro_data_rate() {
    return gyro_rate = 1100 / (1.0 + sensor.getGyroRateDivisor());
}