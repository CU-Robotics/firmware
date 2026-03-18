#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "filters/pid_filter.cpp"

#include "sensors/can/can_manager.hpp"

CANManager can;

int main() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        // wait for serial maybe
    }

    can.init();

    float motor_info[CAN_MAX_MOTORS][4] = {};
    motor_info[0][0] = 2; // controller type c610
    motor_info[0][1] = 1; // physical id 2
    motor_info[0][2] = 0; // can bus 1
    motor_info[0][3] = 0; // motor type not used
    motor_info[1][0] = 2; // controller type c610
    motor_info[1][1] = 1; // physical id 1
    motor_info[1][2] = 1; // can bus 1
    motor_info[1][3] = 0; // motor type not used
    can.configure(motor_info);

    float max_torque = 0;

    // PIDFilter feeder_pid;
    // float gains[4] = { 0.1, 0, 0, 0 };

    while (true) {
        can.write_motor_torque(1, 1, 1.0f);
        can.write_motor_torque(0, 1, 1.0f);

        can.write();
        can.read();

        MotorState state = can.get_motor_state(1, 2);
        if (state.torque > max_torque) {
            max_torque = state.torque;
        }
        Serial.printf("Max Torque: %f\n", max_torque);
        delay(5);
    }

    return 0;
}
