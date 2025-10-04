#include "Dartcam.hpp"

DMAMEM uint16_t frame_buffer[DARTCAM_BUFFER_SIZE];
DMAMEM uint16_t frame_buffer2[DARTCAM_BUFFER_SIZE];

Position position_history[POSITION_HISTORY_SIZE];

Dartcam::Dartcam() : history_index(0), omni(), camera(omni) { }


void Dartcam::init() {
    camera.debug(false);
    uint8_t status = camera.begin(DARTCAM_FRAMESIZE, DARTCAM_FORMAT, DARTCAM_FRAME_RATE, DARTCAM_ID, DARTCAM_USE_GPIO);
    if (!status) {
        Serial.println("camera failed to start");
        //while (1) { }; // halt if camera fails to start
    }
}

void Dartcam::read() {
    camera.readFrame(frame_buffer, sizeof(frame_buffer), frame_buffer2, sizeof(frame_buffer2));
}

void Dartcam::send_frame_serial() {
    for (int i = 0; i < DARTCAM_BUFFER_SIZE; i++) {
        Serial.printf("%.4x", frame_buffer[i]); // print 16-bit hex value
    }
}

void Dartcam::log_position() {
    // future note: consider looking lower in the beginning of flight
    // and middle/higher as flight goes on.

    // currently green threshold filters the image and finds the centroid
    int x_sum = 0, y_sum = 0, green_total = 0;
    int x = 0, y = 0;

    // max green value in RGB565 as an int 63.
    constexpr int green_threshold = 40;

    for (int i = 0; i < DARTCAM_BUFFER_SIZE; i++) {
        int green_component = (frame_buffer[i] & 0x07E0) >> 5; // extract green

        if (green_component >= green_threshold) {
            x_sum += x;
            y_sum += y;
            green_total++;
        }

        x++;
        if (x >= DARTCAM_BUFFER_WIDTH) {
            x = 0;
            y++;
        }
    }

    Position position;
    if (green_total == 0) {
        position = { -1, -1 }; // no green object detected
    } else {
        position = { x_sum / green_total, y_sum / green_total };
    }

    // store in circular buffer
    position_history[history_index] = position;
    history_index = (history_index + 1) % POSITION_HISTORY_SIZE; // increment and wrap around
}

Position Dartcam::get_average_position() {
    int x_sum = 0, y_sum = 0, total = 0;

    for (int i = 0; i < POSITION_HISTORY_SIZE; i++) {
        if (position_history[i].x == -1 || position_history[i].y == -1) {
            continue;
        }

        x_sum += position_history[i].x;
        y_sum += position_history[i].y;
        total++;
    }

    if (total == 0) {
        return { -1, -1 }; // no valid centroids
    }

    return { x_sum / total, y_sum / total };
}

void Dartcam::print_position_history() {
    Serial.printf("History ------\n");
    for (int i = 0; i < POSITION_HISTORY_SIZE; i++) {
        Serial.printf("%2d: %4d, %4d\n", i, position_history[i].x, position_history[i].y);
    }
    Serial.printf("\n");
}
