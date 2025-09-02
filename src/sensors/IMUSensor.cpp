#include "IMUSensor.hpp"
#include "utils/logger.hpp"


// default implementation for printing data
void IMUSensor::print() {
	// Display the temperature data, measured in Celcius
	logger.print("\t\tTemperature ");
	logger.print(get_temperature());
	logger.println(LogDestination::Serial, " deg C");
	// Display the acceleration data, measured in m/s^2)
	logger.print("\t\tAccel X: ");
	logger.print(get_accel_X());
	logger.print(" \tY: ");
	logger.print(get_accel_Y());
	logger.print(" \tZ: ");
	logger.print(get_accel_Z());
	logger.println(LogDestination::Serial, " m/s^2 ");
	// Display gyroscope data, measured in radians/s
	logger.print("\t\tGyro X: ");
	logger.print(get_gyro_X());
	logger.print(" \tY: ");
	logger.print(get_gyro_Y());
	logger.print(" \tZ: ");
	logger.print(get_gyro_Z());
	logger.println(LogDestination::Serial, " radians/s ");
	logger.println();
}