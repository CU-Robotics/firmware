/// @file Arduino.h
/// @brief Minimal Arduino compatibility definitions for host unit tests.
/// @details GPIO and delay calls do nothing, millis() always returns zero, and
/// serial output is written to standard output.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>

#ifndef PI
/// @brief Ratio of a circle's circumference to its diameter.
#define PI 3.14159265358979323846
#endif

/// @brief Arduino output pin mode placeholder.
constexpr int OUTPUT = 1;
/// @brief Arduino logic-high level.
constexpr int HIGH = 1;
/// @brief Arduino logic-low level.
constexpr int LOW = 0;
/// @brief Built-in LED pin number used by the host stub.
constexpr int LED_BUILTIN = 13;

/// @brief Clamp a value to an inclusive range.
/// @tparam T Comparable value type.
/// @param value Value to constrain.
/// @param minimum Inclusive lower bound.
/// @param maximum Inclusive upper bound.
/// @pre minimum must not exceed maximum.
/// @return The constrained value.
template <typename T>
constexpr T constrain(T value, T minimum, T maximum) {
    return std::clamp(value, minimum, maximum);
}

/// @brief Ignore the requested delay in milliseconds and return immediately.
inline void delay(unsigned long) {}
/// @brief Ignore the requested delay in microseconds and return immediately.
inline void delayMicroseconds(unsigned int) {}
/// @brief Ignore the pin number and mode without configuring hardware.
inline void pinMode(std::uint8_t, int) {}
/// @brief Ignore the pin number and output level without changing hardware.
inline void digitalWrite(std::uint8_t, int) {}
/// @brief Provide a fixed time value for host tests.
/// @return Always zero; elapsed time is not simulated.
inline unsigned long millis() { return 0; }

/// @brief Expose the standard infinity check through the Arduino interface.
using std::isinf;
/// @brief Expose the standard NaN check through the Arduino interface.
using std::isnan;

/// @brief Minimal Print base required by the production system logger.
class Print {
public:
    /// @brief Write one byte to the destination.
    /// @param value Byte to write.
    /// @return Number of bytes written.
    virtual size_t write(uint8_t value) = 0;

    /// @brief Write a byte buffer to the destination.
    /// @param buffer Bytes to write.
    /// @param size Number of bytes.
    /// @return Number of bytes written.
    virtual size_t write(const uint8_t* buffer, size_t size) = 0;
};

/// @brief Minimal serial output stub that writes to standard output.
class HardwareSerial {
public:
    /// @brief Write text verbatim to standard output.
    /// @param text Null-terminated text to write.
    void print(const char* text) { std::fputs(text, stdout); }

    /// @brief Write text verbatim without interpreting format specifiers.
    /// @param text Null-terminated text to write.
    void printf(const char* text) {
        std::fputs(text, stdout);
    }

    /// @brief Write formatted text to standard output using std::printf.
    /// @tparam Args Types of the formatting arguments.
    /// @param format Null-terminated printf format string.
    /// @param args Values matching the format specifiers.
    template <typename... Args>
    void printf(const char* format, Args... args) {
        std::printf(format, args...);
    }

    /// @brief Write a value as a double using the %g format, followed by a newline.
    /// @tparam T Value type that can be explicitly converted to double.
    /// @param value Value to print.
    template <typename T>
    void println(const T& value) {
        std::printf("%g\n", static_cast<double>(value));
    }

    /// @brief Write a newline to standard output.
    void println() { std::printf("\n"); }
};

/// @brief Default serial stub, backed by standard output.
inline HardwareSerial Serial;
/// @brief Serial port 1 stub, backed by standard output.
inline HardwareSerial Serial1;
/// @brief Serial port 2 stub, backed by standard output.
inline HardwareSerial Serial2;
/// @brief Serial port 3 stub, backed by standard output.
inline HardwareSerial Serial3;
/// @brief Serial port 4 stub, backed by standard output.
inline HardwareSerial Serial4;
/// @brief Serial port 5 stub, backed by standard output.
inline HardwareSerial Serial5;
/// @brief Serial port 6 stub, backed by standard output.
inline HardwareSerial Serial6;
/// @brief Serial port 7 stub, backed by standard output.
inline HardwareSerial Serial7;
/// @brief Serial port 8 stub, backed by standard output.
inline HardwareSerial Serial8;
