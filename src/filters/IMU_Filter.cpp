#include "IMU_Filter.hpp"

void IMU_filter::init(){
    _icm.init(_icm.SPI);
}

void IMU_filter::read(){
    _imu.accel_X = _icm.get_accel_X();
    _imu.accel_Y = _icm.get_accel_Y();
    _imu.accel_Z = _icm.get_accel_Z();
    _imu.gyro_X = _icm.get_gyro_X();
    _imu.gyro_Y = _icm.get_gyro_Y();
    _imu.gyro_Z = _icm.get_gyro_Z();

    _imu.temperature = _icm.get_temperature();
//Something somthing need to be test
}

IMUData* IMU_filter::getdata(){
    return &_imu;
}