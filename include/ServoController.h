// ServoController.h
#pragma once

class ServoController {
public:
  ServoController();
  void init();
  void setServoAngle(int servoIndex, float angle);
  void setAllServos(float angle1, float angle2, float angle3, float angle4);
  void setAllServosPIDControlled(float angle1, float angle2, float angle3,
                                 float angle4);

private:
};
