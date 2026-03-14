#pragma once

/// @brief state of a set of keyboard keys
struct Keys {
    /// @brief W key status
    uint8_t w = 0;
    /// @brief A key status
    uint8_t a = 0;
    /// @brief S key status
    uint8_t s = 0;
    /// @brief D key status
    uint8_t d = 0;
    /// @brief Shift key status
    uint8_t shift = 0;
    /// @brief Ctrl key status
    uint8_t ctrl = 0;
    /// @brief Q key status
    uint8_t q = 0;
    /// @brief E key status
    uint8_t e = 0;
    /// @brief R key status
    uint8_t r = 0;
    /// @brief F key status
    uint8_t f = 0;
    /// @brief G key status
    uint8_t g = 0;
    /// @brief Z key status
    uint8_t z = 0;
    /// @brief X key status
    uint8_t x = 0;
    /// @brief C key status
    uint8_t c = 0;
    /// @brief V key status
    uint8_t v = 0;
    /// @brief B key status
    uint8_t b = 0;
};

/// @brief Switch Position Enum
enum class SwitchPos : uint32_t{
	INVALID = 0,
	FORWARD,
	BACKWARD,
	MIDDLE
};

