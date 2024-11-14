#include "Dartcam.hpp"

DMAMEM uint16_t frameBuffer[DARTCAM_BUFFER_SIZE];
DMAMEM uint16_t frameBuffer2[DARTCAM_BUFFER_SIZE];

Dartcam::Dartcam() : omni(), camera(omni) { }

void Dartcam::init() {
    camera.debug(false);
    uint8_t status = camera.begin(DARTCAM_FRAMESIZE, DARTCAM_FORMAT, DARTCAM_FRAME_RATE, DARTCAM_ID, DARTCAM_USE_GPIO);
    if (!status) {
        Serial.println("camera failed to start");
        while (1) { }; // halt if camera fails to start
    }
}

void Dartcam::read() {
    camera.readFrame(frameBuffer, sizeof(frameBuffer), frameBuffer2, sizeof(frameBuffer2));
    // Serial.println(frameBuffer[0]);
}

void Dartcam::send_frame_serial() {
    // Serial.write(frameBuffer, sizeof(frameBuffer));
    // print frame buffer as hex to serial
    for (int i = 0; i < DARTCAM_BUFFER_SIZE; i++) {
        Serial.printf("%.4x", frameBuffer[i]);
    }
}

std::pair<int, int> Dartcam::getObjectPosition() {
    // TODO All image capture, object mask/get cords

    // Initialize position with an invalid value
    int x_pos = -1;
    int y_pos = -1;

    return std::make_pair(x_pos, y_pos);
}