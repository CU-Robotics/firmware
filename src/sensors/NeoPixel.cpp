
#include "NeoPixel.hpp"

#include <Arduino.h>

NeoPixel::NeoPixel(uint8_t pin, uint16_t numPixels) : pixels_(numPixels, pin, NEO_GRB + NEO_KHZ800) {}

void NeoPixel::begin() {
    pixels_.begin();
    flashing_ = false;
    ledsOn_ = false;
    pixels_.clear();
    pixels_.show();
}

void NeoPixel::applyColor(uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t color = pixels_.Color(red, green, blue);

    for (uint16_t i = 0; i < pixels_.numPixels(); i++) {
        pixels_.setPixelColor(i, color);
    }

    pixels_.show();
}

void NeoPixel::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    flashing_ = false;
    applyColor(red, green, blue);
}

void NeoPixel::setPixelColor(uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
    if (index >= pixels_.numPixels()) {
        return;
    }

    flashing_ = false;
    pixels_.setPixelColor(index, pixels_.Color(red, green, blue));
}

void NeoPixel::setBrightness(uint8_t brightness) {
    pixels_.setBrightness(brightness);
    pixels_.show();
}

void NeoPixel::clear() {
    flashing_ = false;
    ledsOn_ = false;
    pixels_.clear();
    pixels_.show();
}

void NeoPixel::show() { pixels_.show(); }

void NeoPixel::flash(uint8_t red, uint8_t green, uint8_t blue, uint32_t intervalMs) {
    flashColor_[0] = red;
    flashColor_[1] = green;
    flashColor_[2] = blue;

    flashInterval_ = intervalMs == 0 ? 1 : intervalMs;
    lastFlash_ = millis();

    flashing_ = true;
    ledsOn_ = true;

    // Start with LEDs on
    applyColor(red, green, blue);
}

void NeoPixel::flashForTenSeconds(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue) {
    NeoPixel leds(6, pin);
    leds.begin();

    const uint32_t duration = 10000;
    const uint32_t interval = 500;
    uint32_t startTime = millis();

    while (millis() - startTime < duration) {
        leds.applyColor(red, green, blue);
        delay(interval);

        leds.clear();
        delay(interval);
    }

    leds.clear();
}

void NeoPixel::update() {
    if (!flashing_) {
        return;
    }

    uint32_t now = millis();

    if (now - lastFlash_ < flashInterval_) {
        return;
    }

    lastFlash_ = now;
    ledsOn_ = !ledsOn_;

    if (ledsOn_) {
        applyColor(flashColor_[0], flashColor_[1], flashColor_[2]);
    } else {
        pixels_.clear();
        pixels_.show();
    }
}

void NeoPixel::stopFlashing() { flashing_ = false; }
