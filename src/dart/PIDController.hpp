#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outputMin = -100,
                  float outputMax = 100);

    float calculate(float setpoint, float measuredValue);

    // reset i
    void reset();

private:
    float kp;
    float ki;
    float kd;
    float outputMin;
    float outputMax;

    float previousError;
    float integral;
};

#endif
