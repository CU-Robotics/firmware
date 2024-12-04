#ifndef CAMERA_H
#define CAMERA_H

#include "Teensy_Camera.h"
#include "OV5640.h"
#include <utility>  // Include for std::pair

// define buffer width, height, size
#define DARTCAM_BUFFER_WIDTH 320
#define DARTCAM_BUFFER_HEIGHT 240
#define DARTCAM_BUFFER_SIZE (DARTCAM_BUFFER_WIDTH * DARTCAM_BUFFER_HEIGHT)

#define DARTCAM_ID OV5640a
#define DARTCAM_FRAME_RATE 30
#define DARTCAM_FORMAT RGB565
#define DARTCAM_FRAMESIZE FRAMESIZE_QVGA
#define DARTCAM_USE_GPIO false

/// @brief frame buffer 1 for the camera
extern uint16_t frame_buffer[DARTCAM_BUFFER_SIZE];
/// @brief frame buffer 2 for the camera
// extern uint16_t frameBuffer2[DARTCAM_BUFFER_SIZE];

class Dartcam {
public:
    /// @brief Constructor for the Dartcam object
    Dartcam();

    /// @brief Initialize the camera. Prints an error message and halts if the camera fails to start.
    void init();

    /// @brief Read a frame from the camera and put it into the frame buffers.
    void read();

    /// @brief Send the frame buffer data over serial in hexadecimal format.
    void send_frame_serial();

    /// @brief Get the object's position within the frame.
    /// @return A pair representing the x and y coordinates of the object.
    std::pair<int, int> get_object_position();

private:
    /// @brief ImageSensor object for the OV5640 sensor
    OV5640 omni;

    /// @brief Camera object for the OV5640 sensor. Constructor requires an ImageSensor object.
    Camera camera;
};

extern Dartcam dartcam;

#endif // CAMERA_H
