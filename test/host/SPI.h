/// @file SPI.h
/// @brief Minimal SPI compatibility stubs for host unit tests.
/// @details Configuration, transactions, and transfers do nothing; no hardware
/// state or received data is simulated.
#pragma once

#include <cstddef>
#include <cstdint>

/// @brief Most-significant-bit-first bit order placeholder.
constexpr int MSBFIRST = 1;
/// @brief SPI mode 3 placeholder.
constexpr int SPI_MODE3 = 3;

/// @brief Accept SPI configuration arguments without storing or applying them.
class SPISettings {
public:
    /// @brief Ignore the clock frequency in hertz, bit order, and data mode.
    SPISettings(std::uint32_t, int, int) {}
};

/// @brief No-op SPI bus interface for compiling firmware code on the host.
class SPIClass {
public:
    /// @brief Return without initializing hardware.
    void begin() {}
    /// @brief Ignore the supplied settings without starting a transaction.
    void beginTransaction(const SPISettings&) {}
    /// @brief Return without ending a hardware transaction.
    void endTransaction() {}
    /// @brief Ignore the buffer and byte count, leaving the buffer unchanged.
    void transfer(void*, std::size_t) {}
};

/// @brief Global SPI bus stub used by host unit tests.
inline SPIClass SPI;
