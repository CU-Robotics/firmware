#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <cstdint>
#include "sensors/sensor.hpp"
#include "comms/data/buff_encoder_data.hpp"

constexpr uint32_t read_zero_pos_max_attempts = 10;

// Encoder Registers and Config
constexpr uint32_t MT6835_OP_READ = 0b0011;
constexpr uint32_t MT6835_OP_WRITE = 0b0110;
constexpr uint32_t MT6835_OP_PROG = 0b1100;
constexpr uint32_t MT6835_OP_ZERO = 0b0101;
constexpr uint32_t MT6835_OP_ANGLE = 0b1010;
constexpr uint32_t MT6835_CMD_MASK = 0b111100000000000000000000;
constexpr uint32_t MT6835_ADDR_MASK = 0b000011111111111100000000;
constexpr uint32_t MT6835_DATA_MASK = 0b000000000000000011111111;
constexpr uint32_t MT6835_CPR = 2097152;
constexpr uint32_t MT6835_STATUS_OVERSPEED = 0x01;
constexpr uint32_t MT6835_STATUS_WEAKFIELD = 0x02;
constexpr uint32_t MT6835_STATUS_UNDERVOLT = 0x04;
constexpr uint32_t MT6835_CRC_ERROR = 0x08;
constexpr uint32_t MT6835_WRITE_ACK = 0x55;
constexpr uint32_t MT6835_REG_USERID = 0x001;
constexpr uint32_t MT6835_REG_ANGLE1 = 0x003;
constexpr uint32_t MT6835_REG_ANGLE2 = 0x004;
constexpr uint32_t MT6835_REG_ANGLE3 = 0x005;
constexpr uint32_t MT6835_REG_ANGLE4 = 0x006;
constexpr uint32_t MT6835_REG_ABZ_RES1 = 0x007;
constexpr uint32_t MT6835_REG_ABZ_RES2 = 0x008;
constexpr uint32_t MT6835_REG_ZERO1 = 0x009;
constexpr uint32_t MT6835_REG_ZERO2 = 0x00A;
constexpr uint32_t MT6835_REG_OPTS0 = 0x00A;
constexpr uint32_t MT6835_REG_OPTS1 = 0x00B;
constexpr uint32_t MT6835_REG_OPTS2 = 0x00C;
constexpr uint32_t MT6835_REG_OPTS3 = 0x00D;
constexpr uint32_t MT6835_REG_OPTS4 = 0x00E;
constexpr uint32_t MT6835_REG_OPTS5 = 0x011;
constexpr uint32_t MT6835_REG_NLC_BASE = 0x013;
constexpr uint32_t MT6835_REG_CAL_STATUS = 0x113;
constexpr uint32_t MT6835_BITORDER = MSBFIRST;

/// @brief Driver for the Buff-Encoder
class BuffEncoder : public Sensor {
public:
    /// @brief Constructor for the BuffEncoder class
    /// @param encoder_config configuration data for the encoder
    BuffEncoder(const Cfg::BuffEncoder& encoder_config) : Sensor(), config_data(encoder_config), comms_data(encoder_config.encoder_name) {};

    /// @brief initialize sensor
    void init() override;

    /// @brief Read via SPI the current angle of the encoder
    /// @note Returns and sets m_angle when it reads
    void read() override;

    /// @brief Read the ZERO_POS registers from the encoder
    float read_zero_pos();

    /// @brief Write the ZERO_POS registers to the encoder
    /// @param zero_pos_raw 12-bit value (0-4095) to write
    void write_zero_pos(uint16_t zero_pos_raw);

    /// @brief Send the current data to comms
    void send_to_comms() const override;

    /// @brief Get the angle of the last read function adjusted by the offset
    /// @return Read angle (radians)
    inline float get_angle() const { return m_angle; }

    /// @brief Get the configured name of this encoder
    /// @return The name of this encoder
    inline Cfg::SensorName get_name() const { return config_data.encoder_name; }
 
    /// @brief Print the data for debugging
    void print() const;

    /// @brief Compute CRC8 per MT6835 datasheet spec (poly = X^8 + X^2 + X + 1, MSB first)
    /// @param data Pointer to the 3 bytes covering ANGLE[20:0] + STATUS[2:0] (i.e. data[2], data[3], data[4] from the angle burst read)
    /// @param len Number of bytes (should be 3 for this chip)
    /// @return Computed 8-bit CRC
    uint8_t mt6835_crc8(const uint8_t* data, size_t len) const;

private:

    /// @brief Read angle from the encoder
    float m_angle = 0.f;

    /// @brief True after the first non-empty SPI response has been decoded.
    bool m_has_valid_read = false;

    /// @brief Configuration data for the encoder
    const Cfg::BuffEncoder& config_data;

    /// @brief Data to be send to comms
    BuffEncoderData comms_data;

    /// @brief The SPI settings of the buff encoders
    static const SPISettings m_settings;
};
