#include "../../include/FlightController.h"

FlightController::FlightController(ServoController &sc, IMU &imu, Camera &cam)
    : servoController(sc), imu(imu), camera(cam), currentMode(FIN_HOLD),
      pitchPID(0.1, 0.01, 0.05, -10, 10),
      rollPID(0.1, 0.01, 0.05, -10, 10), // TODO Will need to tune these
      yawPID(0.1, 0.01, 0.05, -10, 10) {}

void FlightController::setControlMode(ControlMode mode) { currentMode = mode; }

void FlightController::update() {
  switch (currentMode) {
  case FIN_HOLD:
    holdFinPosition();
    break;
  case VELOCITY_VECTOR_ALIGNMENT:
    alignToVelocityVector();
    break;
  case GUIDED:
    guided();
    break;
  case FIN_TEST:
    finTestMode();
    break;
  }
}

// Mode 1: Fin Hold
void FlightController::holdFinPosition() {
  //  current fin positions
}

// Mode 2: Velo Vector Stable Flight
void FlightController::alignToVelocityVector() {
  // This is currently just stable flight, will need to be set to the actual
  // velo vector if we want it to fly a true arc.
  float pitchSetpoint = 0.0; // Target pitch angle in degrees
  float yawSetpoint = 0.0;
  float rollSetpoint = 0.0;

  IMUData imuData = imu.readData();
  float currentPitch = imuData.pitch;
  float currentYaw = imuData.yaw;
  float currentRoll = imuData.roll;

  float pitchAdjustment =
      pitchPID.calculate(pitchSetpoint, currentPitch);             // For pitch
  float yawAdjustment = yawPID.calculate(yawSetpoint, currentYaw); // For yaw
  float rollAdjustment =
      rollPID.calculate(rollSetpoint, currentRoll); // For roll stability

  // - Servo 0: Left Elevator
  // - Servo 1: Right Elevator
  // - Servo 2: Left Rudder
  // - Servo 3: Right Rudder

  float leftElevator = pitchAdjustment + rollAdjustment;
  float rightElevator = pitchAdjustment - rollAdjustment;
  float leftRudder = yawAdjustment + rollAdjustment;
  float rightRudder = yawAdjustment - rollAdjustment;

  servoController.setServoAngle(0, leftElevator);
  servoController.setServoAngle(1, rightElevator);
  servoController.setServoAngle(2, leftRudder);
  servoController.setServoAngle(3, rightRudder);
}

// Mode 3: Guided
void FlightController::guided() {
  // Get target pos
  std::pair<int, int> position = camera.getObjectPosition();
  int x_obj = position.first;
  int y_obj = position.second;

  int x_center = 320; // Assumes a 640x480
  int y_center = 240;
  int x_deviation = x_obj - x_center;
  int y_deviation = y_obj - y_center;

  float pitch_adjustment = pitchPID.calculate(0, y_deviation);
  float yaw_adjustment = yawPID.calculate(0, x_deviation);

  // Set servo
  servoController.setAllServos(pitch_adjustment, yaw_adjustment,
                               pitch_adjustment, yaw_adjustment);
}

// Mode 4: Fin Test Mode
void FlightController::finTestMode() {

  static int testStep = 0;
  float testAngle = (testStep % 2 == 0) ? 30 : -30;

  if (testStep % 4 == 0) {
    servoController.setServoAngle(0, testAngle);
  } else if (testStep % 4 == 1) {
    servoController.setServoAngle(1, testAngle);
  } else if (testStep % 4 == 2) {
    servoController.setServoAngle(2, testAngle);
  } else {
    servoController.setServoAngle(3, testAngle);
  }

  testStep++;
}

// helpers
void FlightController::maintainLevelRoll() {
  float rollSetpoint = 0.0;

  IMUData imuData = imu.readData();
  float currentRoll = imuData.roll;

  float rollAdjustment = rollPID.calculate(rollSetpoint, currentRoll);

  // [left_roll, right_roll, other_pitch_yaw]
  servoController.setServoAngle(0, rollAdjustment);
  servoController.setServoAngle(1, -rollAdjustment);
}
