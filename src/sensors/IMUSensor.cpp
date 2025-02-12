#include "IMUSensor.hpp"

IMU_data IMUSensor::get_data(){
    return data;
}

void IMUSensor::fix_raw_data(){
    data.accel_X *= scale_accel;
    data.accel_Y *= scale_accel;
    data.accel_Z *= scale_accel;
    data.gyro_X -= offset_X;
    data.gyro_Y -= offset_Y;
    data.gyro_Z -= offset_Z;
    return;
}

void IMUSensor::print() {
	// Display the temperature data, measured in Celcius
	Serial.print("\t\tTemperature ");
	Serial.print(get_temperature());
	Serial.println(" deg C");
	// Display the acceleration data, measured in m/s^2)
	Serial.print("\t\tAccel X: ");
	Serial.print(get_accel_X());
	Serial.print(" \tY: ");
	Serial.print(get_accel_Y());
	Serial.print(" \tZ: ");
	Serial.print(get_accel_Z());
	Serial.println(" m/s^2 ");
	// Display gyroscope data, measured in radians/s
	Serial.print("\t\tGyro X: ");
	Serial.print(get_gyro_X());
	Serial.print(" \tY: ");
	Serial.print(get_gyro_Y());
	Serial.print(" \tZ: ");
	Serial.print(get_gyro_Z());
	Serial.println(" radians/s ");
	Serial.println();
}

void IMUSensor::calibration_all(){
	Serial.println("Calibrating IMU...");
    float sum_accel_x = 0;
    float sum_accel_y = 0;
    float sum_accel_z = 0;
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    for (int i = 0; i < CALIBRATION_NUM; i++){
        read();
        sum_accel_x += data.accel_X;
        sum_accel_y += data.accel_Y;
        sum_accel_z += data.accel_Z;
        sum_x += data.gyro_X;
        sum_y += data.gyro_Y;
        sum_z += data.gyro_Z;
    }
    float x = sum_accel_x/CALIBRATION_NUM;
    float y = sum_accel_y/CALIBRATION_NUM;
    float z = sum_accel_z/CALIBRATION_NUM;
    scale_accel = SENSORS_GRAVITY_EARTH/sqrt((x*x) + (y*y) + (z*z));
    set_offsets(sum_x / CALIBRATION_NUM, sum_y / CALIBRATION_NUM, sum_z / CALIBRATION_NUM);
    data.accel_X *= scale_accel;
    data.accel_Y *= scale_accel; 
    data.accel_Z *= scale_accel;
    Serial.println("Calibrating Finish...");
}