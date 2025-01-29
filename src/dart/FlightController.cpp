#include "FlightController.hpp"

FlightController::FlightController(ServoController& sc, IMU& imu, Dartcam& dartcam)
    : servoController(sc), imu(imu), dartcam(dartcam), currentMode(FIN_HOLD),
    pitchPID(0.1, 0.01, 0.05, -10, 10),
    rollPID(0.1, 0.01, 0.05, -10, 10), // TODO Will need to tune these
    yawPID(0.1, 0.01, 0.05, -10, 10) {
}

void FlightController::set_control_mode(ControlMode mode) { currentMode = mode; }

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
    case FIN_TEST:
        fin_test_mode();
        break;
    }
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

    IMUData imuData = imu.read_data();
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

    servoController.set_servo_angle(0, leftElevator);
    servoController.set_servo_angle(1, rightElevator);
    servoController.set_servo_angle(2, leftRudder);
    servoController.set_servo_angle(3, rightRudder);
}

// Mode 3: Guided
void FlightController::guided() {
  // Get target pos
    Position position = dartcam.get_average_position();
    int x_obj = position.x;
    int y_obj = position.y;

    int x_center = DARTCAM_BUFFER_WIDTH; // Assumes a 640x480
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
    servoController.set_servo_angle(3, 180);

    static int testStep = 0;
    float testAngle = (testStep % 2 == 0) ? 30 : -30;

    if (testStep % 4 == 0) {
        servoController.set_servo_angle(0, testAngle);
    } else if (testStep % 4 == 1) {
        servoController.set_servo_angle(1, testAngle);
    } else if (testStep % 4 == 2) {
        servoController.set_servo_angle(2, testAngle);
    } else {
        servoController.set_servo_angle(3, testAngle);
    }

    testStep++;
}

// helpers
void FlightController::maintain_level_roll() {
    float rollSetpoint = 0.0;

    IMUData imuData = imu.read_data();
    float currentRoll = imuData.roll;

    float rollAdjustment = rollPID.calculate(rollSetpoint, currentRoll);

    // [left_roll, right_roll, other_pitch_yaw]
    servoController.set_servo_angle(0, rollAdjustment);
    servoController.set_servo_angle(1, -rollAdjustment);
}
