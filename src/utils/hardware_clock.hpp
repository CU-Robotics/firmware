#pragma once
#include "clock.hpp"
#include <Arduino.h>

/// @brief Operates based on micros().
struct HardwareClock : Clock {
    /// @brief Get the current time in micros.
    /// @return The current time in micros.
    uint32_t now_us() const override;
};

/// @brief Get the hardware clock singleton.
/// @returns The hardware clock singleton.
const Clock &hardware_clock();