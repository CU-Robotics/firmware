// DART MAIN
#include <Arduino.h>
#include <TeensyDebug.h>
#include <memory>

// #include "Dartcam.hpp"
#include "IMU.hpp"
// #include "PIDController.hpp"
#include "FlightController.hpp"
#include "ServoController.hpp"
// #include "profiler.hpp"

// Profiler prof;

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

  flightController.set_control_mode(FIN_TEST);

  // main loop
  Serial.println("Entering main loop");

  while (true) {
      
    imu.print_data();
    //flightController.update();
    // servoCont.set_all_servos(180, 180, 180, 180);
    // delay(1000);
    // servoCont.set_all_servos(0, 0, 0, 0);
    // delay(1000);
    //  flightController.update();
    //  flightController.set_control_mode(FIN_TEST);

    // fin.write(180);
    // delay(2000);
    // fin.write(0);
    // delay(2000);

    // uint32_t start_time = micros();
    // Position obj_pos;

    /*
    // update framebuffers
    dartcam.read();

    // log position
    dartcam.log_position();

    obj_pos = dartcam.get_average_position();
    Serial.printf("\n\nPosition: %d, %d\n", obj_pos.x, obj_pos.y);

    dartcam.print_position_history();

    // print fps using elapsed time
    Serial.printf("FPS: %f\n", 1000000.0 / (micros() - start_time));
    */
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
