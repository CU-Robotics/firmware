#pragma once

struct Keys {
    /// W key status
    uint8_t w = 0;
    /// A key status
    uint8_t a = 0;
    /// S key status
    uint8_t s = 0;
    /// D key status
    uint8_t d = 0;
    /// Shift key status
    uint8_t shift = 0;
    /// Ctrl key status
    uint8_t ctrl = 0;
    uint8_t q = 0;
    uint8_t e = 0;
    uint8_t r = 0;
    uint8_t f = 0;
    uint8_t g = 0;
    uint8_t z = 0;
    uint8_t x = 0;
    uint8_t c = 0;
    uint8_t v = 0;
    uint8_t b = 0;
};

/// @brief Switch Position Enum
enum class SwitchPos : uint32_t{
	INVALID = 0,
	FORWARD,
	BACKWARD,
	MIDDLE
};

