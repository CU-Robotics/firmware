// DART MAIN
#include <Arduino.h>
#include <TeensyDebug.h>
#include <memory>

// #include "Dartcam.hpp"
#include "IMU.hpp"
// #include "PIDController.hpp"
#include "FlightController.hpp"
#include "ServoController.hpp"
#include "core_pins.h"
#include "usb_serial.h"
// #include "profiler.hpp"

// Profiler prof;

ServoController servoCont;
IMU imu;
Dartcam dartcam;
FlightController flightController(servoCont, imu, dartcam);

// commet this out to use production main
#define CHOOSE_TEST_MAIN

#ifdef CHOOSE_TEST_MAIN
int main() {
  // setup
  Serial.begin(115200);
  debug.begin(SerialUSB1);

  if (CrashReport) {
    while (!Serial)
      ;
    Serial.println(CrashReport);
    while (1)
      ;
  }

  servoCont.init();
  imu.init();
  dartcam.init();
  flightController.init();

  Serial.println("Entering flight control mode: TEST_GYRO_LEVEL");
  flightController.set_control_mode(TEST_GYRO_LEVEL);

  // main loop
  Serial.println("Entering main loop");

  while (true) {

    // Serial.println("in loop");
    imu.print_data();
    flightController.update();
    //   dartcam.send_frame_serial();
    //    servoCont.set_all_servos(180, 180, 180, 180);
    //    delay(1000);
    //    servoCont.set_all_servos(0, 0, 0, 0);
    //    delay(1000);
    //     flightController.update();
    //     flightController.set_control_mode(FIN_TEST);

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

#else

int main() {
  // setup
  Serial.begin(115200);
  debug.begin(SerialUSB1);

  Serial.println("Starting systems...");
  servoCont.init();
  imu.init();
  dartcam.init();

  // TODO find a better way to controll states?
  bool rail = true;
  bool level = false;
  bool guided = false;
  bool grounded = false;

  Serial.println("Entering flight control mode: RAIL");
  // flightController.set_control_mode(RAIL);
  while (rail) {
    flightController.update();
  }

  level = true;
  flightController.set_control_mode(TEST_GYRO_LEVEL);
  while (level) {
    flightController.update();
  }

  guided = true;
  flightController.set_control_mode(GUIDED);
  while (guided) {
    flightController.update();
  }

  grounded = true;
  // flightController.set_control_mode(GROUNDED);
  while (grounded) {
    flightController.update();
  }
}

#endif

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
