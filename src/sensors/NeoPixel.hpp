#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class NeoPixel {
public:
    NeoPixel(uint8_t pin, uint16_t numPixels);

    void begin();
    void set_color(uint8_t red, uint8_t green, uint8_t blue);
    void update();

private:
    Adafruit_NeoPixel pixels_;

    uint8_t flashColor_[3] = {0, 0, 0};

    uint32_t flashInterval_ = 500;
    uint32_t lastFlash_ = 0;

    bool ledsOn_ = false;
};