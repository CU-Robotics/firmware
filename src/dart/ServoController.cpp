// ServoController.cpp
#include "../../include/ServoController.h"
#include "../../libraries/PWMServo-master/PWMServo.h"

PWMServo fin1, fin2, fin3, fin4;

ServoController::ServoController() {}

void ServoController::init() {
  fin1.attach(9);
  fin2.attach(10);
  fin3.attach(11);
  fin3.attach(12);
}

void ServoController::setServoAngle(int servoIndex, float angle) {
  switch (servoIndex) {
  case (1):
    fin1.write(angle);
  case (2):
    fin2.write(angle);
  case (3):
    fin3.write(angle);
  case (4):
    fin4.write(angle);
  }
}

void ServoController::setAllServos(float angle1, float angle2, float angle3,
                                   float angle4) {
  // Update servo positions
  fin1.write(angle1);
  fin2.write(angle2);
  fin3.write(angle3);
  fin4.write(angle4);
}
