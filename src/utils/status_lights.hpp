#pragma once

#include <stdint.h>
#include "sensors/NeoPixel.hpp"

/// @brief Splits the status NeoPixel strip into one segment per status, each with its own color.
/// @note Segments are laid out in Slot order from DIN, and the strip is split evenly between them.
///       On the 8 pixel strip each slot gets 2 pixels.
class StatusLights {
public:
    /// @brief An RGB color
    struct Color {
        /// @brief Red channel, 0-255
        uint8_t r;
        /// @brief Green channel, 0-255
        uint8_t g;
        /// @brief Blue channel, 0-255
        uint8_t b;
    };

    /// @brief Common colors used by the status slots
    static constexpr Color OFF = {0, 0, 0};
    static constexpr Color RED = {255, 0, 0};
    static constexpr Color YELLOW = {255, 255, 0};
    static constexpr Color GREEN = {0, 255, 0};
    static constexpr Color CYAN = {0, 255, 255};
    static constexpr Color BLUE = {0, 0, 255};
    static constexpr Color MAGENTA = {255, 0, 255};

    /// @brief What each segment of the strip shows
    enum class Slot : uint8_t {
        Safety = 0,     ///< Arming state and the first active safety reason
        Hive,           ///< Whether Hive is running and whether comms is configuring
        Imu,            ///< IMU gyro calibration
        Localization,   ///< Whether the robot is localized on the field
        Count
    };

    /// @brief Construct the status lights. Call begin() before use.
    /// @param pin Teensy data pin connected to the strip's DIN
    /// @param numPixels Number of pixels on the strip
    StatusLights(uint8_t pin, uint16_t numPixels) : strip_(pin, numPixels) {}

    /// @brief Initialize the strip and turn every slot off
    void begin() { strip_.begin(); }

    /// @brief Stage a color for one slot. Call show() to push it to the strip.
    /// @param slot The slot to set
    /// @param color The color to show
    void set(Slot slot, Color color);

    /// @brief Set a slot and push it to the strip right away. Use this outside the main loop, e.g. before a blocking step.
    /// @param slot The slot to set
    /// @param color The color to show
    void set_now(Slot slot, Color color) {
        set(slot, color);
        show();
    }

    /// @brief Push staged colors to the strip, skipping the write if nothing changed
    void show() { strip_.show(); }

private:
    /// @brief The underlying strip
    NeoPixel strip_;
};
