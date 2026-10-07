#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <sys/types.h>
#include <vector>

// Simulated VN100: echo writes and answer model probes only at the device baud.
class HardwareSerial {
public:
    std::deque<uint8_t> input;
    std::vector<std::string> commands;
    unsigned writes = 0;
    unsigned begins = 0;
    uint32_t baud = 0;
    uint32_t device_baud = 921600;
    bool respond = false;
    std::string fail_command;
    void begin(uint32_t value) { baud = value; ++begins; }
    void end() {}
    void setTimeout(unsigned) {}
    size_t available() const { return input.size(); }
    size_t readBytes(uint8_t *data, size_t size) {
        const size_t count = std::min(size, input.size());
        for (size_t i = 0; i < count; ++i) {
            data[i] = input.front();
            input.pop_front();
        }
        return count;
    }
    size_t write(const uint8_t *bytes, size_t size) {
        ++writes;
        const std::string request(reinterpret_cast<const char *>(bytes), size);
        std::string payload = request.substr(1, request.find('*') - 1);
        commands.push_back(payload);
        if (!respond || baud != device_baud ||
            (!fail_command.empty() && payload.find(fail_command) == 0)) {
            return size;
        }
        unsigned reg = 0, value = 0;
        if (std::sscanf(payload.c_str(), "VNRRG,%u", &reg) == 1) {
            if (reg != 1) return size;
            payload = "VNRRG,01,VN-100";
        } else if (std::sscanf(payload.c_str(), "VNWRG,%u,%u", &reg, &value) == 2 && reg == 5) {
            device_baud = value;
        }
        uint8_t checksum = 0;
        for (const char c : payload) checksum ^= static_cast<uint8_t>(c);
        char suffix[6];
        std::snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
        const std::string reply = "$" + payload + suffix;
        input.insert(input.end(), reply.begin(), reply.end());
        return size;
    }
    void println(const char *) {}
    template<class... Args> void printf(const char *, Args...) {}
};

inline HardwareSerial Serial;
inline HardwareSerial Serial3;
inline uint32_t micros() { static uint32_t time = 0; return time += 1000; }
inline void delayMicroseconds(uint32_t) {}
