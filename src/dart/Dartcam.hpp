#ifndef CAMERA_H
#define CAMERA_H

#include "Teensy_Camera.h"
#include "OV5640.h"

// tuning
#define POSITION_HISTORY_SIZE 10

// define buffer width, height, size
#define DARTCAM_BUFFER_WIDTH 320
#define DARTCAM_BUFFER_HEIGHT 240
#define DARTCAM_BUFFER_SIZE (DARTCAM_BUFFER_WIDTH * DARTCAM_BUFFER_HEIGHT)

// camera id
#define DARTCAM_ID OV5640a

// camera settings
#define DARTCAM_FRAME_RATE 60
#define DARTCAM_FORMAT RGB565
#define DARTCAM_FRAMESIZE FRAMESIZE_QVGA
#define DARTCAM_USE_GPIO false

/// @brief Struct representing an (x, y) centroid.
typedef struct {
    int x;  ///< X-coordinate of the centroid
    int y;  ///< Y-coordinate of the centroid
} Position;

/// @brief frame buffer 1 for the camera
extern uint16_t frame_buffer[DARTCAM_BUFFER_SIZE];
/// @brief frame buffer 2 for the camera
extern uint16_t frame_buffer2[DARTCAM_BUFFER_SIZE];

extern Position position_history[POSITION_HISTORY_SIZE];

class Dartcam {
public:
    /// @brief Constructor for the Dartcam object
    Dartcam();

    /// @brief Initialize the camera. Prints an error message and halts if the camera fails to start.
    void init();

    /// @brief Caller for ImageSensor::readFrameCSI. Writes to frame_buffer and frame_buffer2.
    void read();

    /// @brief Send the frame buffer data over serial in hexadecimal format.
    void send_frame_serial();

    void log_position();

    Position get_average_position();

    void print_position_history();
private:
    /// @brief Stores the most recent centroids detected by the camera.
    Position position_history[POSITION_HISTORY_SIZE];

    /// @brief Index for the centroid history circular buffer.
    int history_index;

    /// @brief ImageSensor object for the OV5640 sensor.
    OV5640 omni;

    /// @brief Camera object for the OV5640 sensor. Requires an ImageSensor object.
    Camera camera;
};

extern Dartcam dartcam;

#endif // CAMERA_H
