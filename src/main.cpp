// DART MAIN
#include <Arduino.h>
#include <TeensyDebug.h>

#include "Dartcam.hpp"
#include "FlightController.hpp"
#include "IMU.hpp"
#include "PIDController.hpp"
#include "ServoController.hpp"
#include "profiler.hpp"


Profiler prof;

ServoController servoCont;
IMU imu;
Dartcam dartcam;
FlightController flightController(servoCont, imu, dartcam);

int main() {
    // setup
    Serial.begin(115200);
    debug.begin(SerialUSB1);
    servoCont.init();
    imu.init();
    dartcam.init();

    // main loop
    uint32_t last_time = 0;
    while (true) {
        uint32_t start = micros();

        // update framebuffers
        dartcam.read();

        // dartcam.send_frame_serial(); // debug entire frame

        std::pair<int, int> pos = dartcam.get_object_position();
        Serial.printf("Object position: (%d, %d)\n", pos.first, pos.second); // debug object position

        // // to verify we are double buffering, print a middle slice of both frame buffers each frame
        // for (int i = DARTCAM_BUFFER_SIZE / 2 - 10; i < DARTCAM_BUFFER_SIZE / 2 + 10; i++) {
        //     Serial.printf("%.4x ", frame_buffer[i]);
        // }
        // Serial.printf("\n");
        // for (int i = DARTCAM_BUFFER_SIZE / 2 - 10; i < DARTCAM_BUFFER_SIZE / 2 + 10; i++) {
        //     Serial.printf("%.4x ", frame_buffer2[i]);
        // }
        // Serial.printf("\n\n");
    }

    Serial.println("Done");
}

/* frame over serial for converting into image
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
