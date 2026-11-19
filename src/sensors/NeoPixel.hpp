#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <vector>

/// @brief WS2812 strip driver
class NeoPixel {
public:
    /// @brief Constructor. Call begin() before use.
    /// @param pin Data pin
    /// @param numPixels Pixel count
    NeoPixel(uint8_t pin, uint16_t numPixels);

    /// @brief Initialize the strip, all pixels off
    void begin();

    /// @brief Pixel count
    /// @return Pixel count
    uint16_t num_pixels() const { return static_cast<uint16_t>(colors_.size()); }

    /// @brief Stage one pixel's color. Call show() to send.
    /// @param index Pixel index, 0 nearest DIN
    /// @param red Red channel, 0-255
    /// @param green Green channel, 0-255
    /// @param blue Blue channel, 0-255
    void set_pixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);

    /// @brief Stage a color for a run of pixels. Call show() to send.
    /// @param first First pixel index
    /// @param count Number of pixels
    /// @param red Red channel, 0-255
    /// @param green Green channel, 0-255
    /// @param blue Blue channel, 0-255
    void set_range(uint16_t first, uint16_t count, uint8_t red, uint8_t green, uint8_t blue);

    /// @brief Send staged colors. No-op if nothing changed, since a write blocks interrupts.
    void show();

    /// @brief Set and send every pixel
    /// @param red Red channel, 0-255
    /// @param green Green channel, 0-255
    /// @param blue Blue channel, 0-255
    void set_solid(uint8_t red, uint8_t green, uint8_t blue);

    /// @brief Turn all pixels off
    void off();

private:
    /// @brief Global brightness, 0-255
    static constexpr uint8_t BRIGHTNESS = 32;

    /// @brief Adafruit driver
    Adafruit_NeoPixel pixels_;

    /// @brief Staged color per pixel
    std::vector<uint32_t> colors_;

    /// @brief Whether staged colors differ from the strip
    bool dirty_ = false;
};
