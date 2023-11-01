#include "ICM20649.hpp"
#include "LSM6DSOX.hpp"

ICM20649 icm;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20649 test!");

  // Try to initialize!
  ICM20649::CommunicationProtocol protocol = ICM20649::CommunicationProtocol::SPI;
  icm.init(protocol);

  Serial.println("ICM20649 Found!");
}

void loop() {
  icm.read();
  Serial.print("\t\tTemperature ");
  Serial.print(icm.get_temperature());
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(icm.get_accel_X());
  Serial.print(" \tY: ");
  Serial.print(icm.get_accel_Y());
  Serial.print(" \tZ: ");
  Serial.print(icm.get_accel_Z());
  Serial.println(" m/s^2 ");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.print(icm.get_gyro_X());
  Serial.print(" \tY: ");
  Serial.print(icm.get_gyro_X());
  Serial.print(" \tZ: ");
  Serial.print(icm.get_gyro_Z());
  Serial.println(" radians/s ");
  Serial.println();
  delay(10);
}