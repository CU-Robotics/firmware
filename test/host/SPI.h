#pragma once

#include <cstddef>
#include <cstdint>

constexpr int MSBFIRST = 1;
constexpr int SPI_MODE3 = 3;

class SPISettings {
public:
    SPISettings(std::uint32_t, int, int) {}
};

class SPIClass {
public:
    void begin() {}
    void beginTransaction(const SPISettings&) {}
    void endTransaction() {}
    void transfer(void*, std::size_t) {}
};

inline SPIClass SPI;
