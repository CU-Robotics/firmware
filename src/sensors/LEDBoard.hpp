#ifndef LEDBoard_H
#define LEDBoard_H

#include <FastLED.h>
#include <core_pins.h>
#include "utils/profiler.hpp"

/// @brief The maximum allowed LEDs. You can initialize less than this amount, but not more
constexpr uint32_t LED_MAX = 8;

/// @brief The pin number where the LEDs are connected (compile-time constant).
constexpr uint32_t LED_PIN = 6;

///@class LEDBoard
///@brief A class to control a matrix of LEDs.
class LEDBoard {
public:
    /// @brief Constructs an LEDController object with specified parameters.
    /// @param num_leds The number of LEDs in the matrix. Default is 8.
    /// @param brightness The brightness level of the LEDs (0-255). Default is 100.
    LEDBoard(int num_leds = LED_MAX, uint8_t brightness = 100);

    /// @brief Initialize the LED array and set LED parameters
    void init();
    
    /// @brief Initializes the LED hardware and settings.
    void setup_LEDS();

    
    /// @brief Blinks the first two LEDs on and off ten times.
     /*
     * This function toggles the first two LEDs between on and off states,
     * with a delay to make the blinking visible. It also uses the profiler
     * to measure the performance of the blinking operation.
     */
    void blinkLED();

    
    /// @brief Displays a hexadecimal value on the LED matrix.
    /// @param hexValue A 16-bit hexadecimal value to display on the LEDs.
     /*
     * The function maps each pair of bits in the hexadecimal value to a color
     * and sets the corresponding LED to that color.
     * Black: 00
     * Red: 01
     * Green: 10
     * Blue: 11
     */
    void displayHexOnMatrix(uint16_t hexValue);

    
    /// @brief Internal use - Converts an integer to a 16-bit hexadecimal value.
    /// @param integerValue The integer value to convert.
    /// @return A 16-bit hexadecimal representation of the integer.
    uint16_t convertIntegerToHex(int integerValue);

    
     /// @brief Displays an integer value on the LED matrix.
     /// @param integerValue The integer value to display.
     /*
     * This function converts the integer to a hexadecimal value and then
     * displays it on the LED matrix.
     * 
     * View displayHexOnMatrix for more information
     */
    void displayIntOnMatrix(int integerValue);

    
    /// @brief Sets a specific LED in the matrix to a given color.
    /// @param led_index The index of the LED to set (0-num_leds).
    /// @param color The color to set the LED (using CRGB format).
    void setLedOnMatrix(int led_index, CRGB color);

    
    /// @brief Updates the LED matrix display to reflect any changes made. This function should be called after setting LED colors to update the physical LEDs.
    void updateLEDMatrix();

private:
     /// @brief Maps a 2-bit value to a CRGB color.
     /// @param bits A 2-bit value (0b00 to 0b11).
     /// @return The corresponding CRGB color.
     /*The mapping is as follows:
     * - 0b00: Black
     * - 0b01: Red
     * - 0b10: Green
     * - 0b11: Blue
     */
    CRGB getColorFromBits(uint8_t bits);

    /// @brief Array of LEDs.
    CRGB leds[LED_MAX] = { };
    
    /// @brief The number of LEDs in the matrix.
    const int num_leds;

    /// @brief The brightness level of the LEDs (0-255).
    const uint8_t brightness;
};

#endif 