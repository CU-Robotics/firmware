// DART MAIN
#include <Arduino.h>
#include <TeensyDebug.h>

#include "Dartcam.hpp"
#include "FlightController.hpp"
#include "IMU.hpp"
#include "PIDController.hpp"
#include "ServoController.hpp"

ServoController servoCont;
IMU imu;
Dartcam dartcam;
FlightController flightController(servoCont, imu, dartcam);

int main() {
    Serial.begin(115200);
    debug.begin(SerialUSB1);
    servoCont.init();

    imu.init();
    dartcam.init();

    while (true) {
        flightController.setControlMode(FIN_TEST);
        flightController.update();
    }
}

/* Camera testing code
int main() {
    dartcam.init();
    int frame_count = 0;
    while (true) {
        dartcam.read();
        if (frame_count % 40 == 0) {
            Serial.println("BEGIN\n");
            dartcam.send_frame_serial();
            Serial.println("\nEND");
        }
        delay(500);
        frame_count++;
    }
    return 0;
}
*/


