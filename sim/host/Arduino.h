#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstddef>

#define PI 3.14159265358979323846
namespace firmware_sim_host { inline thread_local uint64_t time_us = 0; }
inline uint32_t micros() { return static_cast<uint32_t>(firmware_sim_host::time_us); }
inline uint32_t millis() { return static_cast<uint32_t>(firmware_sim_host::time_us / 1000); }
template<class T> constexpr T constrain(T x, T lo, T hi) { return std::clamp(x, lo, hi); }
using std::isnan;
using std::isinf;
class HardwareSerial {
public:
    template<class... Args> void printf(const char* format, Args... args) { std::fprintf(stderr, format, args...); }
    void println(const char* value) { std::fprintf(stderr, "%s\n", value); }
    void println() { std::fputc('\n', stderr); }
};
inline HardwareSerial Serial;
