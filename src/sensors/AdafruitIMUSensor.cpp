#include "IMUSensor.hpp"

// default implementation for printing data
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