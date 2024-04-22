#ifndef BUFF_ENCODER_H
#define BUFF_ENCODER_H

#include <Arduino.h>
#include <SPI.h>

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

// Chip Select pins for the two encoders
constexpr int YAW_BUFF_CS = 37;
constexpr int PITCH_BUFF_CS = 36;

/// @brief Driver for the Buff-Encoder
class BuffEncoder {
public:
    /// @brief 
    /// @param 
    BuffEncoder() {};

    /// @brief Initialize the encoder object with the specific Chip Select pin
    /// @param cs The Chip Select pin
    BuffEncoder(int cs) : m_CS(cs) {};

    /// @brief initialize sensor with new cs(if needed)
    /// @param cs input Chip Select pin
    void init(int cs) { m_CS = cs; }

    /// @brief Read via SPI the current angle of the encoder
    /// @return Read angle (radians)
    /// @note Returns and sets m_angle when it reads
    float read();

    /// @brief Get the angle of the last read function
    /// @return Read angle (radians)
    inline float get_angle() const { return m_angle; }


private:
    /// @brief Stored Chip Select pin
    int m_CS = 0;

    /// @brief Read angle from the encoder
    float m_angle = 0.f;

    /// @brief The SPI settings of the buff encoders
    static const SPISettings m_settings;

};

#endif