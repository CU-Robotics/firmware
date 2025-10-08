#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "sensors/can/can_manager.hpp"

CANManager can;

int main() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        // wait for serial maybe
    }

    can.init();

    // id, bus ,
    float motor_info[CAN_MAX_MOTORS][4] = {};
    motor_info[0][0] = 1; // controller type c610
    motor_info[0][1] = 2; // physical id 2
    motor_info[0][2] = 1; // can bus 1
    motor_info[0][3] = 0; // motor type not used
    can.configure(motor_info);

    while (true) {
        can.write_motor_torque(1, 2, 0.3f);
        can.write();
        can.read();
        delay(300);
    }

    return 0;
}
