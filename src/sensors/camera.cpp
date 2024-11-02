#include "camera.hpp"

DMAMEM uint8_t frameBuffer[DARTCAM_BUFFER_SIZE];
DMAMEM uint8_t frameBuffer2[DARTCAM_BUFFER_SIZE];

Dartcam::Dartcam() : omni(), camera(omni) { }

void Dartcam::init() {
    uint8_t status = camera.begin(DARTCAM_FRAMESIZE, DARTCAM_FORMAT, DARTCAM_FRAME_RATE, DARTCAM_ID, DARTCAM_USE_GPIO);
    if (!status) {
        Serial.println("camera failed to start");
        while (1) { }; // halt if camera fails to start
    }
}

void Dartcam::read() {
    camera.readFrame(frameBuffer, sizeof(frameBuffer), frameBuffer2, sizeof(frameBuffer2));
}

void Dartcam::process_frame() {

}