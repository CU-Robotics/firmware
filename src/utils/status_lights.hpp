#pragma once

#include <stdint.h>
#include "sensors/NeoPixel.hpp"

/// @brief Status strip split evenly into one segment per Slot, in order from DIN
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

    /// @brief All channels off
    static constexpr Color OFF = {0, 0, 0};
    /// @brief Red
    static constexpr Color RED = {255, 0, 0};
    /// @brief Yellow
    static constexpr Color YELLOW = {255, 255, 0};
    /// @brief Green
    static constexpr Color GREEN = {0, 255, 0};
    /// @brief Cyan
    static constexpr Color CYAN = {0, 255, 255};
    /// @brief Blue
    static constexpr Color BLUE = {0, 0, 255};
    /// @brief Magenta
    static constexpr Color MAGENTA = {255, 0, 255};

    /// @brief Strip segments
    enum class Slot : uint8_t {
        Safety = 0,     ///< Arming state
        Hive,           ///< Hive running / configuring
        Imu,            ///< IMU calibration
        Localization,   ///< Localization
        Count
    };

    /// @brief Constructor. Call begin() before use.
    /// @param pin Data pin
    /// @param numPixels Pixel count
    StatusLights(uint8_t pin, uint16_t numPixels) : strip_(pin, numPixels) {}

    /// @brief Initialize the strip, all off
    void begin() { strip_.begin(); }

    /// @brief Stage a slot's color. Call show() to send.
    /// @param slot The slot to set
    /// @param color The color to show
    void set(Slot slot, Color color);

    /// @brief Set a slot and send immediately
    /// @param slot The slot to set
    /// @param color The color to show
    void set_now(Slot slot, Color color) {
        set(slot, color);
        show();
    }

    /// @brief Send staged colors
    void show() { strip_.show(); }

private:
    /// @brief Underlying strip
    NeoPixel strip_;
};
