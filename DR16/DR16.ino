#include "CAN_Manager.hpp"
#include "DR16.hpp"

CAN_Manager can;
DR16 sensor;

uint32_t currTime = 0;
uint32_t dt = 0;

void setup() {
  Serial5.clear();
	Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);

  can.init();
  sensor.Init();
}

void loop() {
  delay(1);
  while(can.read()) {}

  sensor.Read();

  can.zero();

  can.write();
}