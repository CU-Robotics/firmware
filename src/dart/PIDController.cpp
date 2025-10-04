#include "PIDController.hpp"

PIDController::PIDController(float kp, float ki, float kd, float outputMin,
                             float outputMax)
    : kp(kp), ki(ki), kd(kd), outputMin(outputMin), outputMax(outputMax),
    previousError(0), integral(0) {
}

float PIDController::calculate(float setpoint, float measuredValue) {
  // error
    float error = setpoint - measuredValue;

    // P
    float proportional = kp * error;

    // I
    integral += error;
    float integralTerm = ki * integral;

    // D
    float derivative = kd * (error - previousError);

    // output
    float output = proportional + integralTerm + derivative;

    // limits
    if (output > outputMax)
        output = outputMax;
    else if (output < outputMin)
        output = outputMin;

    previousError = error;

    return output;
}

void PIDController::reset() {
    integral = 0;
    previousError = 0;
}
