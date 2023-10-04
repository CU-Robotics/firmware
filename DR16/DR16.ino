#include "CAN_Manager.hpp"
#include "DR16.hpp"

CAN_Manager can;
DR16 sensor;

void setup() {
  Serial5.clear();
	Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);

  can.init();
  sensor.Init();
}

void loop() {
  delay(1);
  //while(can.read()) {}

  sensor.Read();

  Serial.print("Right Stick X: ");
  Seirla.print(sensor.get_r_stick_x());
  Serial.print("\tRight Stick Y: ");
  Serial.print(sensor.get_r_stick_y());
  Serial.print("\tLeft Stick X: ");
  Seirla.print(sensor.get_l_stick_x());
  Serial.print("\tLeft Stick Y: ");
  Serial.print(sensor.get_l_stick_y());
  Serial.print("\tSwitch L: ");
  Serial.print(sensor.get_l_switch());
  Serial.println("\tSwitch R: ");

  can.zero();

  //can.write();
}