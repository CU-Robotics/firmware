#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "Dartcam.hpp"
#include "ICM20649.hpp"
#include "IMU_filter.hpp"
#include "PIDController.hpp"
#include "ServoController.hpp" 
enum ControlMode {
  FIN_HOLD,
  VELOCITY_VECTOR_ALIGNMENT,
  GUIDED,
  TEST_FIN,
  TEST_GYRO_LEVEL
};

class FlightController {
public:
  FlightController(ServoController &servoController, ICM20649 &imu, IMU_filter &imuF, Dartcam &dartcam);
  void update();
  void set_control_mode(ControlMode mode);
  void init();

private:
  ServoController &servoController;
  ICM20649 &imu;
  IMU_filter &imuF;
  Dartcam &dartcam;
  ControlMode currentMode;

  PIDController pitchPID;
  PIDController rollPID;
  PIDController yawPID;

  void hold_fin_position();
  void guided();
  void align_to_velocity_vector();
  void fin_test_mode();
  void maintain_level_roll();
  void test_gyro_level();
};

#endif
