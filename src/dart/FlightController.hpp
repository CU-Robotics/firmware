#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "Dartcam.hpp"
#include "IMU.hpp"
#include "PIDController.hpp"
#include "ServoController.hpp"

enum ControlMode { FIN_HOLD, VELOCITY_VECTOR_ALIGNMENT, GUIDED, FIN_TEST };

class FlightController {
public:
    FlightController(ServoController& servoController, IMU& imu, Dartcam& dartcam);
    void update();
    void setControlMode(ControlMode mode);

private:
    ServoController& servoController;
    IMU& imu;
    Dartcam& dartcam;
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

#endif
