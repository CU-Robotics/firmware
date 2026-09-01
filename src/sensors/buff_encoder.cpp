#include "buff_encoder.hpp"
#include "comms/data/sendable.hpp"
#include "utils/safety.hpp"
#include "utils/system_log.hpp"
#include <SPI.h>

const SPISettings BuffEncoder::m_settings = SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);


uint32_t read_count = 0;
void BuffEncoder::init() {
    // set the SPI pins to the correct mode
    pinMode(config_data.spi_cs, OUTPUT);
    digitalWrite(config_data.spi_cs, HIGH); // set CS high to start
	tx_buffer[0] = (MT6835_OP_ANGLE << 4);
	tx_buffer[1] = MT6835_REG_ANGLE1;
	// Flush the cache to RAM
    arm_dcache_flush_delete(tx_buffer, sizeof(tx_buffer));

    while (read_zero_pos() != 0.0f && read_count < read_zero_pos_max_attempts) {
        write_zero_pos(0);
        read_count++;
    }
    read_count = 0;
}
void BuffEncoder::isr_start_transfer(EventResponderRef spi_event) {
	SPI1.beginTransaction(m_settings);
	digitalWrite(config_data.spi_cs, LOW);

	SPI1.transfer(tx_buffer, rx_buffer, 6, spi_event); //after testing make this an assert_or_safety_procedure()
}
void BuffEncoder::isr_stop_transfer(EventResponderRef spi_event) {
	digitalWrite(config_data.spi_cs, HIGH);
    SPI1.endTransaction();
    arm_dcache_delete(rx_buffer, 32);
	
}
void BuffEncoder::read() {
	if (shared_dma_flag != nullptr && *shared_dma_flag == true) {
        return; 
    }
    // Serial.printf("Pin: %u, Sending Buff Encoder read command\n", config_data.spi_cs);
    
    uint8_t status = rx_buffer[4] & 0x07;
    uint8_t crc_received = rx_buffer[5];
    uint8_t crc_computed  = mt6835_crc8(&rx_buffer[2], 3);

    if (crc_received != crc_computed) {
        SystemLog.error(Subsystem::SENSORS,"Pin: %u, MT6835 CRC mismatch\n", config_data.spi_cs);
        return;
    }
    if (status & MT6835_STATUS_UNDERVOLT) { 
        SystemLog.error(Subsystem::SENSORS,"Pin: %u, MT6835 undervoltage detected\n", config_data.spi_cs); 
        return;
    }
    if (status & MT6835_STATUS_WEAKFIELD) { 
        SystemLog.error(Subsystem::SENSORS,"Pin: %u, MT6835 weak field detected\n", config_data.spi_cs); 
        return; 
    }

    // convert received angle into radians
	int raw_angle = (rx_buffer[2] << 13) | (rx_buffer[3] << 5) | (rx_buffer[4] >> 3);
	float radians = raw_angle / (float)MT6835_CPR * (3.14159265f * 2.0f);

    // assign angle
    m_angle = radians;

    // Serial.printf("Buff Encoder %u - angle: %f\n", static_cast<uint32_t>(config_data.encoder_name), m_angle);

    comms_data.m_angle = m_angle;

    // read_zero_pos();
}

void BuffEncoder::write_zero_pos(uint16_t zero_pos_raw) {
    if (zero_pos_raw > 0x0FFF) {
        SystemLog.error(Subsystem::SENSORS,"Pin: %u, ZERO_POS value out of range: %u\n", config_data.spi_cs, zero_pos_raw);
        return;
    }

    uint8_t zero_pos_high = (zero_pos_raw >> 4) & 0xFF; // ZERO_POS[11:4] -> full byte for 0x009
    uint8_t zero_pos_low  = zero_pos_raw & 0x0F;          // ZERO_POS[3:0] -> top nibble of 0x00A

    // --- Read back current 0x00A first, so we don't clobber Z_EDGE / Z_PUL_WID[2:0] ---
    uint8_t tx_read[3] = { 0 };
    tx_read[0] = (MT6835_OP_READ << 4) | ((MT6835_REG_ZERO2 >> 8) & 0x0F);
    tx_read[1] = MT6835_REG_ZERO2 & 0xFF;
    tx_read[2] = 0x00;

    SPI1.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI1.transfer(tx_read, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI1.endTransaction();

    uint8_t current_00A = tx_read[2];
    uint8_t z_edge_and_pulwid = current_00A & 0x0F; // preserve Z_EDGE (bit3) + Z_PUL_WID[2:0] (bits2:0)

    delayMicroseconds(1);

    // --- Write 0x009: ZERO_POS[11:4] ---
    uint8_t tx009[3];
    tx009[0] = (MT6835_OP_WRITE << 4) | ((MT6835_REG_ZERO1 >> 8) & 0x0F);
    tx009[1] = MT6835_REG_ZERO1 & 0xFF;
    tx009[2] = zero_pos_high;

    SPI1.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI1.transfer(tx009, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI1.endTransaction();

    delayMicroseconds(1);

    // --- Write 0x00A: ZERO_POS[3:0] | Z_EDGE | Z_PUL_WID[2:0] (preserved) ---
    uint8_t tx00A[3];
    tx00A[0] = (MT6835_OP_WRITE << 4) | ((MT6835_REG_ZERO2 >> 8) & 0x0F);
    tx00A[1] = MT6835_REG_ZERO2 & 0xFF;
    tx00A[2] = (zero_pos_low << 4) | z_edge_and_pulwid;

    SPI1.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI1.transfer(tx00A, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI1.endTransaction();

    SystemLog.info(Subsystem::SENSORS,"Pin: %u, wrote ZERO_POS = 0x%03X (%u)\n",
                  config_data.spi_cs, zero_pos_raw, zero_pos_raw);
}

float BuffEncoder::read_zero_pos() {
    // --- Read register 0x009: ZERO_POS[11:4] ---
    uint8_t tx009[3] = { 0 };
    tx009[0] = (MT6835_OP_READ << 4) | ((MT6835_REG_ZERO1 >> 8) & 0x0F); // command nibble + A11:A8
    tx009[1] = MT6835_REG_ZERO1 & 0xFF;                                  // A7:A0
    tx009[2] = 0x00;                                                     // dummy byte to clock out data

    SPI1.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI1.transfer(tx009, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI1.endTransaction();

    uint8_t zero_pos_high = tx009[2]; // ZERO_POS[11:4]

    delayMicroseconds(10); // small gap between transactions, adjust if needed

    // --- Read register 0x00A: ZERO_POS[3:0] | Z_EDGE | Z_PUL_WID[2:0] ---
    uint8_t tx00A[3] = { 0 };
    tx00A[0] = (MT6835_OP_READ << 4) | ((MT6835_REG_ZERO2 >> 8) & 0x0F);
    tx00A[1] = MT6835_REG_ZERO2 & 0xFF;
    tx00A[2] = 0x00;

    SPI1.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI1.transfer(tx00A, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI1.endTransaction();

    uint8_t reg00A = tx00A[2];
    uint8_t zero_pos_low = (reg00A >> 4) & 0x0F; // ZERO_POS[3:0]

    uint16_t zero_pos_raw = (static_cast<uint16_t>(zero_pos_high) << 4) | zero_pos_low; // 12-bit value, 0-4095

    if (zero_pos_raw != 0) {
        SystemLog.info(Subsystem::SENSORS,"Pin: %u, ZERO_POS raw = 0x%03X (%u), degrees = %.3f\n",
                  config_data.spi_cs, zero_pos_raw, zero_pos_raw,
                  zero_pos_raw * (360.0f / 4096.0f));
    }

    return zero_pos_raw * (360.0f / 4096.0f);
}

void BuffEncoder::send_to_comms() const {
    Comms::Sendable<BuffEncoderData> sendable;
    sendable.data = comms_data;
    sendable.send_to_comms();
}

void BuffEncoder::print() const {
    Serial.printf("Buff Encoder:\n\t");
    Serial.println(get_angle());
}
void BuffEncoder::print_live_data() {
    // Note: casting get_name() to int so it prints the enum number
    Serial.printf(" [Buff Encoder %d] Angle (rad): %8.4f\n", 
                  (int)get_name(), get_angle());
}

void BuffEncoder::bind_dma_flag(const volatile bool* flag_ptr) {
        shared_dma_flag = flag_ptr;
}
uint8_t BuffEncoder::mt6835_crc8(const uint8_t* data, size_t len) const {
    constexpr uint8_t poly = 0x07; // X^8 + X^2 + X + 1
    uint8_t crc = 0x00;            // datasheet does not state a seed; 0x00 is the typical default for this polynomial family

    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}
