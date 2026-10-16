#include "hardware_clock.hpp"
#include <Arduino.h>

uint32_t HardwareClock::now_us() const { return micros(); }

const Clock &hardware_clock() {
    // create singleton
    const static HardwareClock instance{};
    return instance;
}