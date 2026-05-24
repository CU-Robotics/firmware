#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <cstdint>
#include "sensors/sensor.hpp"
#include "comms/data/buff_encoder_data.hpp"


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
	
	/// @copydoc AdafruitIMUSensor::request_read()
    //void request_read() override;
	
    /// @brief Read via SPI the current angle of the encoder
    /// @note Returns and sets m_angle when it reads
    void read() override;

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
	
	/// @brief Prints a formatted dashboard of live Buff Encoder values
    void print_live_data() override;
	
    /// @brief Enables this encoder and start transfer
	/// @param spi_event is global event handler from sensor_manager
    void isr_start_transfer(EventResponderRef spi_event);
	
    /// @brief Disable this encoder and send DMA cache to memory
	/// @param spi_event is global event handler from sensor_manager
    void isr_stop_transfer(EventResponderRef spi_event);
	
    /// @brief Binds local dma flag with global sensor_manager flag
	/// @param flag_ptr is shared dma flag from sensor_manager
	void bind_dma_flag(const volatile bool* flag_ptr);

private:

    /// @brief Read angle from the encoder
    float m_angle = 0.f;

    /// @brief Configuration data for the encoder
    const Cfg::BuffEncoder& config_data;

    /// @brief Data to be send to comms
    BuffEncoderData comms_data;

    /// @brief The SPI settings of the buff encoders
    static const SPISettings m_settings;
	
	/// @brief Buffer of transmitted data to the buff encoders
    alignas(32) uint8_t tx_buffer[32];
	
	/// @brief Buffer of recieved data from the buff encoders
    alignas(32) uint8_t rx_buffer[32];

	/// @brief Pointer to the SensorManager's active transfer flag
    const volatile bool* shared_dma_flag;
	// Could/should this be a shared pointer??
};
