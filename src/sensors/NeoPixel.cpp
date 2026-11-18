#include "NeoPixel.hpp"
#include <Arduino.h>

NeoPixel::NeoPixel(uint8_t pin, uint16_t numPixels)
    : pixels_(numPixels, pin, NEO_GRB + NEO_KHZ800), colors_(numPixels, 0) {}

void NeoPixel::begin() {
    pixels_.begin();
    pixels_.setBrightness(BRIGHTNESS);

    // Always write once so the strip matches colors_ (all off) after power up
    pixels_.clear();
    pixels_.show();
    dirty_ = false;
}

void NeoPixel::set_pixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
    if (index >= colors_.size()) {
        return;
    }

    uint32_t color = Adafruit_NeoPixel::Color(red, green, blue);
    if (colors_[index] == color) {
        return;
    }

    colors_[index] = color;
    pixels_.setPixelColor(index, color);
    dirty_ = true;
}

void NeoPixel::set_range(uint16_t first, uint16_t count, uint8_t red, uint8_t green, uint8_t blue) {
    for (uint16_t i = 0; i < count; i++) {
        set_pixel(first + i, red, green, blue);
    }
}

void NeoPixel::show() {
    if (!dirty_) {
        return;
    }

    pixels_.show();
    dirty_ = false;
}

void NeoPixel::set_solid(uint8_t red, uint8_t green, uint8_t blue) {
    set_range(0, num_pixels(), red, green, blue);
    show();
}

void NeoPixel::off() {
    set_solid(0, 0, 0);
}
