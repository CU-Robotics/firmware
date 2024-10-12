#ifndef CAMERA_H
#define CAMERA_H

#include "Teensy_Camera.h"
#include "OV5640.h"

#define CAMERA_ID OV5640a

struct Dartcam {
    uint8_t frameBuffer[160 * 120];
    uint8_t frameBuffer2[160 * 120];

    OV5640 omni;
    Camera camera;

    Dartcam() : omni(), camera(omni) { }

    void init() {
        Serial.println("initializing camera");
        uint8_t status = camera.begin(FRAMESIZE_VGA, JPEG, 30, CAMERA_ID, false);
        if (!status) {
            Serial.println("camera failed to start");
            while (1) { } // Halt if camera fails to start
        }
    }

    void read() {
        camera.readFrame(frameBuffer, sizeof(frameBuffer), frameBuffer2, sizeof(frameBuffer2));

        Serial.println("frame captured");

        Serial.println(frameBuffer[0]);
        Serial.println(frameBuffer[1]);
        Serial.println(frameBuffer[200]);
    }
};

#endif // CAMERA_H
