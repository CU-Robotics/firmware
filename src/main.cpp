#include "../include/Camera.h"
#include "../include/FlightController.h"
#include "../include/IMU.h"
#include "../include/ServoController.h"

ServoController servoCont;
IMU imu;
Camera camera;
FlightController flightController(servoCont, imu, camera);

void setup() {
  // init
  servoCont.init();
  imu.init();
  camera.init();
}

void loop() {
  flightController.setControlMode(FIN_TEST);
  flightController.update();
}
