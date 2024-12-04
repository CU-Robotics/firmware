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
    const int WIDTH = DARTCAM_BUFFER_WIDTH;
    const int HEIGHT = DARTCAM_BUFFER_HEIGHT;

    int x_sum = 0;
    int y_sum = 0;
    int count = 0;

    const int max_green_value = 63; // 6 bits for green in RGB565
    const int threshold_value = static_cast<int>(max_green_value * 0.9); // 90% of max green
    const uint16_t green_mask = 0x07E0; // Mask to extract green component

    for (int i = 0; i < DARTCAM_BUFFER_SIZE; i++) {
        int green_component = (frame_buffer[i] & green_mask) >> 5;

        if (green_component >= threshold_value) {
            int x = i % WIDTH;
            int y = i / WIDTH;
            x_sum += x;
            y_sum += y;
            count++;
        }
    }

    int x_pos = -1;
    int y_pos = -1;

    if (count > 0) {
        x_pos = x_sum / count;
        y_pos = y_sum / count;
    }

    return std::make_pair(x_pos, y_pos);
}
