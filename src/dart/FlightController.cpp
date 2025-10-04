#include "FlightController.hpp"
#include "ICM20649.hpp"
#include "IMU.hpp"
#include "IMUSensor.hpp"
#include "IMU_filter.hpp"
#include "core_pins.h"
#include "usb_serial.h"
#include "wiring.h"

#define SETPOINT 45

IMU_data imu_data;

FlightController::FlightController(ServoController &sc, ICM20649 &imu,
                                   IMU_filter &imuF, Dartcam &dartcam)
    : servoController(sc), imu(imu), imuF(imuF), dartcam(dartcam),
      currentMode(FIN_HOLD), pitchPID(0.1, 0.01, 0.05, -10, 10),
      rollPID(0.1, 0.01, 0.05, -10, 10), // TODO Will need to tune these
      yawPID(0.1, 0.01, 0.05, -10, 10) {}

void FlightController::set_control_mode(ControlMode mode) {
  currentMode = mode;
}

void FlightController::update() {
  switch (currentMode) {
  case FIN_HOLD:
    hold_fin_position();
    break;
  case VELOCITY_VECTOR_ALIGNMENT:
    align_to_velocity_vector();
    break;
  case GUIDED:
    guided();
    break;
  case TEST_FIN:
    fin_test_mode();
    break;
  case TEST_GYRO_LEVEL:
    test_gyro_level();
    break;
  }
}

void FlightController::init() {
  servoController.set_all_servos(SETPOINT, SETPOINT, SETPOINT, SETPOINT);
  delay(1000);
}

// Mode 1: Fin Hold
void FlightController::hold_fin_position() {
  //  current fin positions
}

// Mode 2: Velo Vector Stable Flight
void FlightController::align_to_velocity_vector() {
  // This is currently just stable flight, will need to be set to the actual
  // velo vector if we want it to fly a true arc.

  float pitchSetpoint = 0.0; // Target pitch angle in degrees
  float yawSetpoint = 0.0;
  float rollSetpoint = 0.0;

  imu.read();
  imu.fix_raw_data();

  imuF.step_EKF_6axis(imu.get_data());

  IMU_data *filtered_data = imuF.get_filter_data();
  float currentRoll = (filtered_data->roll * RAD_TO_DEG);
  float currentPitch = (filtered_data->pitch * RAD_TO_DEG);
  float currentYaw = (filtered_data->yaw * RAD_TO_DEG);

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

  servoController.set_servo_angle(1, leftElevator);
  servoController.set_servo_angle(2, rightElevator);
  servoController.set_servo_angle(3, leftRudder);
  servoController.set_servo_angle(4, rightRudder);
}

// Mode 3: Guided
void FlightController::guided() {
  // Get target pos
  Position position = dartcam.get_average_position();
  int x_obj = position.x;
  int y_obj = position.y;

  int x_center = DARTCAM_BUFFER_WIDTH; // Assumes a 320x240
  int y_center = DARTCAM_BUFFER_HEIGHT;
  int x_deviation = x_obj - x_center;
  int y_deviation = y_obj - y_center;

  float pitch_adjustment = pitchPID.calculate(0, y_deviation);
  float yaw_adjustment = yawPID.calculate(0, x_deviation);

  // Set servo
  servoController.set_all_servos(pitch_adjustment, yaw_adjustment,
                                 pitch_adjustment, yaw_adjustment);
}

// Mode 4: Fin Test Mode
void FlightController::fin_test_mode() {

  Serial.println("trying to spin servo");
  // servoController.set_servo_angle(3, 180);

  static int testStep = 10;
  float testAngle = (testStep % 2 == 0) ? 30 : -30;

  if (testStep % 4 == 0) {
    servoController.set_servo_angle(1, testAngle - SETPOINT);
  } else if (testStep % 4 == 1) {
    servoController.set_servo_angle(2, testAngle - SETPOINT);
  } else if (testStep % 4 == 2) {
    servoController.set_servo_angle(3, testAngle - SETPOINT);
  } else {
    servoController.set_servo_angle(4, testAngle - SETPOINT);
  }

  testStep++;
}

void FlightController::test_gyro_level() {

  imu.read();
  imu.fix_raw_data();

  imuF.step_EKF_6axis(imu.get_data());

  IMU_data *filtered_data = imuF.get_filter_data();

  // float pitchSetpoint = 0.0; // Target pitch angle in degrees
  //  float yawSetpoint = 0.0;
  // float rollSetpoint = SETPOINT;

  float currentRoll = (filtered_data->roll * RAD_TO_DEG);
  // float currentYaw = imu_data.;
  // float currentRoll = imu_data.k_roll;

  // Serial.println();
  // servoController.set_servo_angle(2, currentRoll + rollSetpoint);
  // servoController.set_servo_angle(3, (currentRoll  + rollSetpoint));
  servoController.set_all_servos(
      -1 * (currentRoll + SETPOINT), -1 * (currentRoll + SETPOINT),
      -1 * (currentRoll + SETPOINT), -1 * (currentRoll + SETPOINT));
  /**

    float pitchAdjustment =
        pitchPID.calculate(pitchSetpoint, currentPitch); // For pitch
    // float yawAdjustment = yawPID.calculate(yawSetpoint, currentYaw); // For
    yaw float rollAdjustment = rollPID.calculate(rollSetpoint, currentRoll); //
    For roll stability

    // - Servo 0: Left Elevator
    // - Servo 1: Right Elevator
    // - Servo 2: Left Rudder
    // - Servo 3: Right Rudder

    float leftElevator = pitchAdjustment + rollAdjustment;
    float rightElevator = pitchAdjustment - rollAdjustment;
    // float leftRudder = yawAdjustment + rollAdjustment;
    // float rightRudder = yawAdjustment - rollAdjustment;

    servoController.set_servo_angle(0, leftElevator);
    servoController.set_servo_angle(1, rightElevator);
    // servoController.set_servo_angle(2, leftRudder);
    // servoController.set_servo_angle(3, rightRudder);
    */
}

// helpers
void FlightController::maintain_level_roll() {
  // float rollSetpoint = 0.0;

  /*
  imu_data = *imuf.get_filter_data();
  float currentRoll = imu_data.roll;

  float rollAdjustment = rollPID.calculate(rollSetpoint, currentRoll);

  // [left_roll, right_roll, other_pitch_yaw]
  servoController.set_servo_angle(0, rollAdjustment);
  servoController.set_servo_angle(1, -rollAdjustment);*/
}
