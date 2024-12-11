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
        // calculate fps
        // Serial.printf("FPS: %f\n", 1000000.0 / last_time);

        uint32_t start = micros();

        // update framebuffers
        dartcam.read();
        std::pair<int, int> pos = dartcam.get_object_position();

        Serial.printf("Object position: (%d, %d)\n", pos.first, pos.second);

        // debug finding general direction of target
        if (pos.first > DARTCAM_BUFFER_WIDTH / 2) {
            Serial.println("(left)");
        } else if (pos.first < DARTCAM_BUFFER_WIDTH / 2) {
            Serial.println("(right)");
        }
        
        if (pos.second > DARTCAM_BUFFER_HEIGHT / 2) {
            Serial.println("(up)");
        } else if (pos.second < DARTCAM_BUFFER_HEIGHT / 2) {
            Serial.println("(down)");
        }

        last_time = micros() - start;
    }

    Serial.println("Done");
    
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
