#include "NeoPixel.hpp"
#include <Arduino.h>

NeoPixel::NeoPixel(uint8_t pin, uint16_t numPixels)
    : pixels_(numPixels, pin, NEO_GRB + NEO_KHZ800) {}

void NeoPixel::begin() {
    pixels_.begin();
    pixels_.clear();
    pixels_.show();

    ledsOn_ = false;
    lastFlash_ = millis();
}

void NeoPixel::set_color(uint8_t red, uint8_t green, uint8_t blue) {
    flashColor_[0] = red;
    flashColor_[1] = green;
    flashColor_[2] = blue;
}

void NeoPixel::update() {
    uint32_t now = millis();

    if (now - lastFlash_ < flashInterval_) {
        return;
    }

    lastFlash_ = now;
    ledsOn_ = !ledsOn_;

    if (ledsOn_) {
        uint32_t color = pixels_.Color(
            flashColor_[0],
            flashColor_[1],
            flashColor_[2]
        );

        for (uint16_t i = 0; i < pixels_.numPixels(); i++) {
            pixels_.setPixelColor(i, color);
        }
    } else {
        pixels_.clear();
    }

    pixels_.show();
}