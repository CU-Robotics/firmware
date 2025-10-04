// ServoController.cpp
#include "ServoController.hpp"
#include "PWMServo.hpp"

PWMServo fin1, fin2, fin3, fin4;

ServoController::ServoController() {}

void ServoController::init() {
  fin1.attach(2);
  fin2.attach(3);
  fin3.attach(4);
  fin4.attach(5);
}

void ServoController::set_servo_angle(int servoIndex, float angle) {
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

void ServoController::set_all_servos(float angle1, float angle2, float angle3,
                                     float angle4) {
  // Update servo positions
  fin1.write(angle1);
  fin2.write(angle2);
  fin3.write(angle3);
  fin4.write(angle4);
}
