#include "read_write.hpp"

CAN_Manager can;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  can.init();
}

void loop() {
  Serial.println("loop begin");
  delay(10);

  while (can.read()) { Serial.print("r"); }
  Serial.println();

  can.zero();

  can.print_output();

  can.write();
}

