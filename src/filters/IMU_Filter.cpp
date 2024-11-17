#include "IMU_Filter.hpp"

void IMU_filter::init(){
    pinMode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);
    SPI.begin();
    _icm.init(_icm.SPI);
    _icm.set_gyro_range(4000);
    calibrate_imu();

}

void IMU_filter::read(){
    _icm.read();
    _imu.accel_X = _icm.get_accel_X() * _imu.accel_scale;
    _imu.accel_Y = _icm.get_accel_Y() * _imu.accel_scale; 
    _imu.accel_Z = _icm.get_accel_Z() * _imu.accel_scale;
    _imu.alpha_X = _icm.get_gyro_X();
    _imu.alpha_Y = _icm.get_gyro_Y();
    _imu.alpha_Z = _icm.get_gyro_Z();
    float dt = timer.delta();
    _imu.angle_X += dt * _icm.get_gyro_X(); 
    _imu.angle_Y += dt * _icm.get_gyro_Y();
    _imu.temperature = _icm.get_temperature();
//Something somthing need to be test
}

void IMU_filter::print(){
    // Display the temperature data, measured in Celcius
	Serial.print("\t\tTemperature ");
	Serial.print(_imu.temperature);
	Serial.println(" deg C");
	// Display the acceleration data, measured in m/s^2)
	Serial.print("\t\tAccel X: ");
	Serial.print(_imu.accel_X);
	Serial.print(" \tY: ");
	Serial.print(_imu.accel_Y);
	Serial.print(" \tZ: ");
	Serial.print(_imu.accel_Z);
	Serial.println(" m/s^2 ");
	// Display gyroscope data, measured in radians/s
	Serial.print("\t\tGyro X: ");
	Serial.print(_imu.alpha_X);
	Serial.print(" \tY: ");
	Serial.print(_imu.alpha_Y);
	Serial.print(" \tZ: ");
	Serial.print(_imu.alpha_Z);
	Serial.println(" radians/s ");
	Serial.println(_imu.angle_Y);
    Serial.println("_imu.angle_Y");
    Serial.println(_imu.angle_X);
    Serial.println("_imu.angle_X");
}   

IMUData* IMU_filter::getdata(){
    return &_imu;
}

void IMU_filter::calibrate_imu(){
    Serial.println("Calibrating IMU...");
    float sum_accel_x = 0;
    float sum_accel_y = 0;
    float sum_accel_z = 0;
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    for (int i = 0; i < CALIBRATION_NUM; i++){
        _icm.read();
        sum_accel_x += _icm.get_accel_X();
        sum_accel_y += _icm.get_accel_Y();
        sum_accel_z += _icm.get_accel_Z();
        sum_x += _icm.get_gyro_X();
        sum_y += _icm.get_gyro_Y();
        sum_z += _icm.get_gyro_Z();
    }
    float x = sum_accel_x/CALIBRATION_NUM;
    float y = sum_accel_y/CALIBRATION_NUM;
    float z = sum_accel_z/CALIBRATION_NUM;
    _imu.accel_scale = SENSORS_GRAVITY_EARTH/sqrt((x*x) + (y*y) + (z*z));
    _icm.set_offsets(sum_x / CALIBRATION_NUM, sum_y / CALIBRATION_NUM, sum_z / CALIBRATION_NUM);
    _imu.angle_Y=-atanf(x / sqrt(y*y + z*z));
    _imu.angle_X=atanf(y / sqrt(x*x + z*z));


}