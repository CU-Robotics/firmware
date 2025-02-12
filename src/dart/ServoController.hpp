// ServoController.h
#pragma once

class ServoController {
public:
  ServoController();
  void init();
  void set_servo_angle(int servoIndex, float angle);
  void set_all_servos(float angle1, float angle2, float angle3, float angle4);
  void setAllServosPIDControlled(float angle1, float angle2, float angle3, float angle4);

private:
};
