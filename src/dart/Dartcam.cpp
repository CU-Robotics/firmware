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

void Dartcam::green_threshold(uint16_t frame_buffer[DARTCAM_BUFFER_SIZE]) {
    int green_mask = 0x07E0; // RGB565 (0b0000011111100000)
    int green_threshold = 0x07E0 * 0.9; // 90% of the maximum green value

    for (int i = 0; i < DARTCAM_BUFFER_SIZE; i++) {
        // extract the green component
        int green_value = frame_buffer[i] & green_mask;

        // theshold the green component
        if (green_value < green_threshold) {
            frame_buffer[i] = 0x0000; // set to black if below threshold
        } else {
            frame_buffer[i] = green_value; // keep the green component
        }
    }
}

std::pair<int, int> Dartcam::get_object_position() {

    // TODO
    int max_green = 0;

    // initialize position with an invalid value
    int x_pos = -1;
    int y_pos = -1;

    return std::make_pair(x_pos, y_pos);
}