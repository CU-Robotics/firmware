#include "Dartcam.hpp"

DMAMEM uint16_t frame_buffer[DARTCAM_BUFFER_SIZE];
// DMAMEM uint16_t frameBuffer2[DARTCAM_BUFFER_SIZE];

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
    camera.readFrame(frame_buffer, sizeof(frame_buffer));
    // Serial.println(frame_buffer[0]);
}

void Dartcam::send_frame_serial() {
    // Serial.write(frame_buffer, sizeof(frame_buffer));
    // print frame buffer as hex to serial
    for (int i = 0; i < DARTCAM_BUFFER_SIZE; i++) {
        Serial.printf("%.4x", frame_buffer[i]);
    }
}

std::pair<int, int> Dartcam::get_object_position() {
    int x_sum = 0;
    int y_sum = 0;
    int green_total = 0;
    int x = 0;
    int y = 0;

    // iterate over 1D frame buffer
    for (int i = 0; i < DARTCAM_BUFFER_SIZE; i++) {
        // isolate green component from RGB565
        int green_component = (frame_buffer[i] & 0x07E0) >> 5;

        // check green component against threshold
        if (green_component >= static_cast<int>((63 * 0.9))) {
            // add coordinate to sum for centroid calculation
            x_sum += x;
            y_sum += y;
            green_total++;
        }

        // update row and column, 1D style
        x++;
        if (x >= DARTCAM_BUFFER_WIDTH) {
            x = 0;
            y++;
        }
    }

    // return centroid of object
    return std::make_pair(x_sum / green_total, y_sum / green_total);
}
