// DART MAIN
#include <Arduino.h>
#include <TeensyDebug.h>

#include "FlightController.hpp"
#include "IMUSensor.hpp"
#include "IMU_filter.hpp"
#include "ServoController.hpp"
#include "core_pins.h"
#include "git_info.h"
#include "sensors/ICM20649.hpp"
#include "usb_serial.h"
#include "wiring.h"

// #include "core_pins.h"
//  #include "usb_serial.h"
//   #include "profiler.hpp"
//

// Profiler prof;

ServoController servoCont;
// IMU imu;
ICM20649 imu;
IMU_filter imuF;
IMU_data imuData;
Dartcam dartcam;

FlightController flightController(servoCont, imu, imuF, dartcam);

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

  SPI.begin();
  imu.init(imu.SPI);
  imu.set_gyro_range(4000);
  imu.calibration_all();
  imuF.init_EKF_6axis(imu.get_data());

  servoCont.init();
  imuF.init_EKF_6axis(imuData);
  // dartcam.init();
  flightController.init();

  // flightController.set_control_mode(TEST_FIN);

  Serial.println("Entering flight control mode: TEST_GYRO_LEVEL");
  flightController.set_control_mode(TEST_GYRO_LEVEL);

  // main loop
  Serial.println("Entering main loop");

  while (true) {

    // imu.read();
    // imu.fix_raw_data();

    // imuF.step_EKF_6axis(imu.get_data());
    // IMU_data *filtered_data = imuF.get_filter_data();

    // Serial.println(filtered_data->pitch * RAD_TO_DEG);

    // Serial.print(filtered_data->pitch);
    // Serial.print(filtered_data->roll);
    // Serial.println(filtered_data->yaw);

    // delay(1000);
    //  Serial.println("in loop");
    //  imuData = *imuF.get_filter_data();
    //  Serial.print(imuData.pitch);
    flightController.update();
    //    dartcam.send_frame_serial();
    //     servoCont.set_all_servos(180, 180, 180, 180);
    //     delay(1000);
    //     servoCont.set_all_servos(0, 0, 0, 0);
    //     delay(1000);
    //      flightController.update();
    //      flightController.set_control_mode(FIN_TEST);

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
