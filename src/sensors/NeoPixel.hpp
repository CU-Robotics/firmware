
#pragma once

#include <Adafruit_NeoPixel.h>
#include <cstdint>

class NeoPixel {
public:
    NeoPixel(uint8_t pin, uint16_t numPixels);

    void begin();

    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setPixelColor(uint16_t index, uint8_t red,
                       uint8_t green, uint8_t blue);
    void setBrightness(uint8_t brightness);

    void clear();
    void show();

    // Flash one color on and off
    void flash(uint8_t red, uint8_t green, uint8_t blue,
               uint32_t intervalMs);

    // Flash a specified RGB color for 10 seconds
    static void flashForTenSeconds(uint8_t red, uint8_t green, uint8_t blue);
    
    // Call regularly to update flashing
    void update();

    // Stop flashing, retaining the current LED state
    void stopFlashing();

private:
    Adafruit_NeoPixel pixels_;

    uint8_t flashColor_[3] = {0, 0, 0};

    uint32_t flashInterval_ = 500;
    uint32_t lastFlash_ = 0;

    bool flashing_ = false;
    bool ledsOn_ = false;

    void applyColor(uint8_t red, uint8_t green, uint8_t blue);
};
