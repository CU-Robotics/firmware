// camera.cpp
#include "../../include/Camera.h"

Camera::Camera()
    : frameWidth(640), frameHeight(480), hueMin(50), hueMax(70),
      saturationMin(100), saturationMax(255), valueMin(100), valueMax(255) {}

void Camera::init() {
  // init camera
}

std::pair<int, int> Camera::getObjectPosition() {
  // TODO All image capture, object mask/get cords

  // Initialize position with an invalid value
  int x_pos = -1;
  int y_pos = -1;

  return std::make_pair(x_pos, y_pos);
}
