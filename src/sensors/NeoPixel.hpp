#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <vector>

/// @brief Drives a WS2812 strip. Pixels can be set one at a time or all at once.
/// @note set_pixel() and set_range() only stage colors. Call show() to push them to the strip.
class NeoPixel {
public:
    /// @brief Construct a strip driver. Call begin() before use.
    /// @param pin Teensy data pin connected to the strip's DIN
    /// @param numPixels Number of pixels on the strip
    NeoPixel(uint8_t pin, uint16_t numPixels);

    /// @brief Initialize the strip and turn all pixels off
    void begin();

    /// @brief Get the number of pixels on the strip
    /// @return Pixel count
    uint16_t num_pixels() const { return static_cast<uint16_t>(colors_.size()); }

    /// @brief Stage a color for one pixel. Does nothing if index is out of range.
    /// @param index Pixel index, 0 is the pixel closest to DIN
    /// @param red Red channel, 0-255
    /// @param green Green channel, 0-255
    /// @param blue Blue channel, 0-255
    void set_pixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);

    /// @brief Stage a color for a run of pixels. Pixels past the end of the strip are ignored.
    /// @param first Index of the first pixel
    /// @param count Number of pixels to set
    /// @param red Red channel, 0-255
    /// @param green Green channel, 0-255
    /// @param blue Blue channel, 0-255
    void set_range(uint16_t first, uint16_t count, uint8_t red, uint8_t green, uint8_t blue);

    /// @brief Push staged colors to the strip.
    /// @note Skips the write if nothing changed since the last show(), since writing disables interrupts for ~30us per pixel.
    void show();

    /// @brief Set every pixel to one color and push it to the strip now
    /// @param red Red channel, 0-255
    /// @param green Green channel, 0-255
    /// @param blue Blue channel, 0-255
    void set_solid(uint8_t red, uint8_t green, uint8_t blue);

    /// @brief Turn all pixels off and push it to the strip now
    void off();

private:
    /// @brief Global brightness, 0-255. Colors are scaled by this before being sent.
    static constexpr uint8_t BRIGHTNESS = 32;

    /// @brief Underlying Adafruit driver for the strip
    Adafruit_NeoPixel pixels_;

    /// @brief Packed, unscaled color staged for each pixel, used to skip redundant writes
    std::vector<uint32_t> colors_;

    /// @brief Whether a staged color differs from what was last sent to the strip
    bool dirty_ = false;
};
