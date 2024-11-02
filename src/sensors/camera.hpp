#ifndef CAMERA_H
#define CAMERA_H

#include "Teensy_Camera.h"
#include "OV5640.h"

#define DARTCAM_BUFFER_SIZE 38400 // 10240
#define DARTCAM_ID OV5640a
#define DARTCAM_FRAME_RATE 30
#define DARTCAM_FORMAT RGB565
#define DARTCAM_FRAMESIZE FRAMESIZE_QQVGA
#define DARTCAM_USE_GPIO false

/// @brief frame buffer 1 for the camera
extern uint8_t frameBuffer[ ];
/// @brief frame buffer 2 for the camera
extern uint8_t frameBuffer2[ ];

struct Dartcam {
    /// @brief ImageSensor object for the OV5640 sensor
    OV5640 omni;
    /// @brief Camera object for the OV5640 sensor. Constructor requires an ImageSensor object.
    Camera camera;

    /// @brief Constructor for the Dartcam object
    Dartcam();

    /// @brief Initialize the camera. Prints an error message and halts if the camera fails to start.
    void init();

    /// @brief Read a frame from the camera and put it into the frame buffers.
    void read();

    void process_frame();
};

#endif // CAMERA_H