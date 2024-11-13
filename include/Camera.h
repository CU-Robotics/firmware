// Camera.h
#pragma once
#include <utility>

class Camera {
public:
  Camera();
  void init();
  std::pair<int, int> getObjectPosition();

private:
  int frameWidth;  // Camera frame width
  int frameHeight; // Camera frame height

  // object detection
  int hueMin, hueMax;
  int saturationMin, saturationMax;
  int valueMin, valueMax;
};
