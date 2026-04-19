#include "AdafruitIMUSensor.hpp"

// default implementation for printing data
void AdafruitIMUSensor::print() {
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
void AdafruitIMUSensor::print_live_data() {
    Serial.printf("=== LIVE ADAFRUIT IMU DATA ===\n");
    
    // Print temperature
    Serial.printf(" Temperature: %5.2f °C\n", get_temperature());
    Serial.println("------------------------------------------------");
    
    // Print Acceleration and Gyro data
    Serial.printf(" Accel (m/s^2) | X: %6.2f | Y: %6.2f | Z: %6.2f\n", 
                  get_accel_X(), get_accel_Y(), get_accel_Z());
                  
    Serial.printf(" Gyro (rad/s)  | X: %6.2f | Y: %6.2f | Z: %6.2f\n", 
                  get_gyro_X(), get_gyro_Y(), get_gyro_Z());
}
