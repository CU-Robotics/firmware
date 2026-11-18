#include "NeoPixel.hpp"
#include <Arduino.h>

NeoPixel::NeoPixel(uint8_t pin, uint16_t numPixels)
    : pixels_(numPixels, pin, NEO_GRB + NEO_KHZ800) {}

void NeoPixel::begin() {
    pixels_.begin();
    pixels_.setBrightness(BRIGHTNESS);
    off();
}

void NeoPixel::set_solid(uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t color = pixels_.Color(red, green, blue);
    if (hasColor_ && color == currentColor_) {
        return;
    }

    pixels_.fill(color);
    pixels_.show();

    currentColor_ = color;
    hasColor_ = true;
}

void NeoPixel::off() {
    set_solid(0, 0, 0);
}
