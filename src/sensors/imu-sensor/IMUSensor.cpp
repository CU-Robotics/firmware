#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <IMUSensor.hpp>

float IMUSensor::get_temperature() { return temperature; };

float IMUSensor::get_accel_X() { return accel_X; };
float IMUSensor::get_accel_Y() { return accel_X; };
float IMUSensor::get_accel_Z() { return accel_X; };

float IMUSensor::get_gyro_X() { return gyro_X; };
float IMUSensor::get_gyro_Y() { return gyro_Y; };
float IMUSensor::get_gyro_Z() { return gyro_Z; };