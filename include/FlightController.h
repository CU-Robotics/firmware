// FlightControll.h
#pragma once
#include "Camera.h"
#include "IMU.h"
#include "PIDController.h"
#include "ServoController.h"

enum ControlMode { FIN_HOLD, VELOCITY_VECTOR_ALIGNMENT, GUIDED, FIN_TEST };

class FlightController {
public:
  FlightController(ServoController &servoController, IMU &imu, Camera &camera);
  void update();
  void setControlMode(ControlMode mode);

private:
  ServoController &servoController;
  IMU &imu;
  Camera &camera;
  ControlMode currentMode;

  PIDController pitchPID;
  PIDController rollPID;
  PIDController yawPID;

  void holdFinPosition();
  void guided();
  void alignToVelocityVector();
  void finTestMode();
  void maintainLevelRoll();
};
