#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

/// @brief Drives a WS2812 strip that shows one solid color for robot status
class NeoPixel {
public:
    /// @brief Construct a strip driver. Call begin() before use.
    /// @param pin Teensy data pin connected to the strip's DIN
    /// @param numPixels Number of pixels on the strip
    NeoPixel(uint8_t pin, uint16_t numPixels);

    /// @brief Initialize the strip and turn all pixels off
    void begin();

    /// @brief Set every pixel to one color and push it to the strip now.
    /// @note Skips the write if the color is unchanged, since show() disables interrupts for ~30us per pixel.
    /// @param red Red channel, 0-255
    /// @param green Green channel, 0-255
    /// @param blue Blue channel, 0-255
    void set_solid(uint8_t red, uint8_t green, uint8_t blue);

    /// @brief Turn all pixels off
    void off();

private:
    /// @brief Global brightness, 0-255. Colors are scaled by this before being sent.
    static constexpr uint8_t BRIGHTNESS = 32;

    /// @brief Underlying Adafruit driver for the strip
    Adafruit_NeoPixel pixels_;

    /// @brief Packed color last sent to the strip, used to skip redundant writes
    uint32_t currentColor_ = 0;

    /// @brief Whether currentColor_ holds a color that was actually sent
    bool hasColor_ = false;
};
