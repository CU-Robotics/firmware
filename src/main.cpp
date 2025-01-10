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
    while (true) {
        uint32_t start_time = micros();
        Position obj_pos;

        // update framebuffers
        dartcam.read();

        // log position
        dartcam.log_position();

        obj_pos = dartcam.get_average_position();
        Serial.printf("\n\nPosition: %d, %d\n", obj_pos.x, obj_pos.y);

        dartcam.print_position_history();

        // print fps using elapsed time
        Serial.printf("FPS: %f\n", 1000000.0 / (micros() - start_time));
    }
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
