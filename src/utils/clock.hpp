#pragma once
#include <cstdint>

/// @brief Abstract clock class. Provides simple interface for getting current time in micros.
struct Clock {
    /// @brief Virtual destructor so child classes destruct properly.
    virtual ~Clock() = default;
    /// @brief Get the current time in micros.
    /// @return The current time in micros.
    virtual uint32_t now_us() const = 0;
};