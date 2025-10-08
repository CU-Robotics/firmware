#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "sensors/can/can_manager.hpp"

CANManager can;


int main() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        // wait for the USB serial connection when available
    }

    can.init();

    while (true) {
        can.write_motor_torque(0, 0, 1.0f);
        can.write();
        can.read();
        delay(5);
    }

    return 0;
}
