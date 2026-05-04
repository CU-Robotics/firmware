#include "LSM6DSOX.hpp"
#include "comms/data/sendable.hpp"

// initialize LSM
void LSM6DSOX::init_impl() {
    // start I2C communication (SPI not supported on LSM6DOX+LIS3MDL hardware)
    sensor.begin_I2C();

    set_accel_range(config.accel_range);
    set_gyro_range(config.gyro_range);
}

void LSM6DSOX::read_impl() {
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

    //copy the values to the data struct
    comms_data.accel_X = accel_X;
    comms_data.accel_Y = accel_Y;
    comms_data.accel_Z = accel_Z;
    comms_data.gyro_X = gyro_X;
    comms_data.gyro_Y = gyro_Y;
    comms_data.gyro_Z = gyro_Z;
    comms_data.temperature = temperature;

}

void LSM6DSOX::send_to_comms_impl() const {
    Comms::Sendable<LsmSensorData> sendable;
    sendable.data = comms_data;
    sendable.send_to_comms();
}

void LSM6DSOX::set_accel_range(Cfg::LsmImuAccelRange range) {
    switch (range) {
        case Cfg::LsmImuAccelRange::A_2G:
            sensor.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
            break;
        case Cfg::LsmImuAccelRange::A_4G:
            sensor.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            break;
        case Cfg::LsmImuAccelRange::A_8G:
            sensor.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
            break;
        case Cfg::LsmImuAccelRange::A_16G:
            sensor.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
            break;
    }
}

void LSM6DSOX::set_gyro_range(Cfg::LsmImuGyroRange range) {
    switch (range) {
        case Cfg::LsmImuGyroRange::DPS125:
            sensor.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
            break;
        case Cfg::LsmImuGyroRange::DPS250:
            sensor.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
            break;
        case Cfg::LsmImuGyroRange::DPS500:
            sensor.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
            break;
        case Cfg::LsmImuGyroRange::DPS1000:
            sensor.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
            break;
        case Cfg::LsmImuGyroRange::DPS2000:
            sensor.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            break;
    }
}
