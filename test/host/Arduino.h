#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#ifndef PI
#define PI 3.14159265358979323846
#endif

constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int LED_BUILTIN = 13;

template <typename T>
constexpr T constrain(T value, T minimum, T maximum) {
    return std::clamp(value, minimum, maximum);
}

inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(std::uint8_t, int) {}
inline void digitalWrite(std::uint8_t, int) {}
inline unsigned long millis() { return 0; }

using std::isinf;
using std::isnan;

class HardwareSerial {
public:
    void printf(const char* text) {
        std::fputs(text, stdout);
    }

    template <typename... Args>
    void printf(const char* format, Args... args) {
        std::printf(format, args...);
    }

    template <typename T>
    void println(const T& value) {
        std::printf("%g\n", static_cast<double>(value));
    }

    void println() { std::printf("\n"); }
};

inline HardwareSerial Serial;
inline HardwareSerial Serial1;
inline HardwareSerial Serial2;
inline HardwareSerial Serial3;
inline HardwareSerial Serial4;
inline HardwareSerial Serial5;
inline HardwareSerial Serial6;
inline HardwareSerial Serial7;
inline HardwareSerial Serial8;
