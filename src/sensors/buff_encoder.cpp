#include "buff_encoder.hpp"
#include "comms/data/sendable.hpp"

const SPISettings BuffEncoder::m_settings = SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);


int read_count = 0;
void BuffEncoder::init() {
    // set the SPI pins to the correct mode
    pinMode(config_data.spi_cs, OUTPUT);
    digitalWrite(config_data.spi_cs, HIGH); // set CS high to start

    for (int i = 0; i < 5 && !m_has_valid_read; i++) {
        delayMicroseconds(100);
        read();
        read_count++;
    }

    while (read_zero_pos() != 0.0f) {
        write_zero_pos(0);
    }
}

void BuffEncoder::read() {
    uint8_t data[6] = { 0 }; // transact 48 bits

    // set the operation
    data[0] = (MT6835_OP_ANGLE << 4);
    data[1] = MT6835_REG_ANGLE1;

    // Serial.printf("Pin: %u, Sending Buff Encoder read command\n", config_data.spi_cs);
    // do the SPI transfer
    SPI.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI.transfer(data, 6);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI.endTransaction();

    uint8_t status = data[4] & 0x07;
    uint8_t crc_received = data[5];
    uint8_t crc_computed  = mt6835_crc8(&data[2], 3);

    if (crc_received != crc_computed) {
        Serial.printf("Pin: %u, MT6835 CRC mismatch\n", config_data.spi_cs);
    }
    if (status & MT6835_STATUS_UNDERVOLT) { Serial.printf("Pin: %u, MT6835 undervoltage detected\n", config_data.spi_cs); }
    if (status & MT6835_STATUS_WEAKFIELD) { Serial.printf("Pin: %u, MT6835 weak field detected\n", config_data.spi_cs); }

    const bool empty_response = data[2] == 0 && data[3] == 0 && data[4] == 0 && data[5] == 0;
    if (!m_has_valid_read && empty_response) {
        Serial.printf("Pin: %u, MT6835 empty response, likely not ready yet\n", config_data.spi_cs);
        return;
    }

    // convert received angle into radians
    int raw_angle = (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
    float radians = raw_angle / (float)MT6835_CPR * (3.14159265 * 2.0);

    // assign angle
    m_angle = radians;
    m_has_valid_read = true;

    // Serial.printf("Buff Encoder %u - angle: %f\n", static_cast<uint32_t>(config_data.encoder_name), m_angle);

    comms_data.m_angle = m_angle;

    // read_zero_pos();
}

void BuffEncoder::write_zero_pos(uint16_t zero_pos_raw) {
    if (zero_pos_raw > 0x0FFF) {
        Serial.printf("Pin: %u, ZERO_POS value out of range: %u\n", config_data.spi_cs, zero_pos_raw);
        return;
    }

    uint8_t zero_pos_high = (zero_pos_raw >> 4) & 0xFF; // ZERO_POS[11:4] -> full byte for 0x009
    uint8_t zero_pos_low  = zero_pos_raw & 0x0F;          // ZERO_POS[3:0] -> top nibble of 0x00A

    // --- Read back current 0x00A first, so we don't clobber Z_EDGE / Z_PUL_WID[2:0] ---
    uint8_t tx_read[3] = { 0 };
    tx_read[0] = (MT6835_OP_READ << 4) | ((MT6835_REG_ZERO2 >> 8) & 0x0F);
    tx_read[1] = MT6835_REG_ZERO2 & 0xFF;
    tx_read[2] = 0x00;

    SPI.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI.transfer(tx_read, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI.endTransaction();

    uint8_t current_00A = tx_read[2];
    uint8_t z_edge_and_pulwid = current_00A & 0x0F; // preserve Z_EDGE (bit3) + Z_PUL_WID[2:0] (bits2:0)

    delayMicroseconds(1);

    // --- Write 0x009: ZERO_POS[11:4] ---
    uint8_t tx009[3];
    tx009[0] = (MT6835_OP_WRITE << 4) | ((MT6835_REG_ZERO1 >> 8) & 0x0F);
    tx009[1] = MT6835_REG_ZERO1 & 0xFF;
    tx009[2] = zero_pos_high;

    SPI.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI.transfer(tx009, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI.endTransaction();

    delayMicroseconds(1);

    // --- Write 0x00A: ZERO_POS[3:0] | Z_EDGE | Z_PUL_WID[2:0] (preserved) ---
    uint8_t tx00A[3];
    tx00A[0] = (MT6835_OP_WRITE << 4) | ((MT6835_REG_ZERO2 >> 8) & 0x0F);
    tx00A[1] = MT6835_REG_ZERO2 & 0xFF;
    tx00A[2] = (zero_pos_low << 4) | z_edge_and_pulwid;

    SPI.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI.transfer(tx00A, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI.endTransaction();

    Serial.printf("Pin: %u, wrote ZERO_POS = 0x%03X (%u)\n",
                  config_data.spi_cs, zero_pos_raw, zero_pos_raw);
}

float BuffEncoder::read_zero_pos() {
    // --- Read register 0x009: ZERO_POS[11:4] ---
    uint8_t tx009[3] = { 0 };
    tx009[0] = (MT6835_OP_READ << 4) | ((MT6835_REG_ZERO1 >> 8) & 0x0F); // command nibble + A11:A8
    tx009[1] = MT6835_REG_ZERO1 & 0xFF;                                  // A7:A0
    tx009[2] = 0x00;                                                     // dummy byte to clock out data

    SPI.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI.transfer(tx009, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI.endTransaction();

    uint8_t zero_pos_high = tx009[2]; // ZERO_POS[11:4]

    delayMicroseconds(10); // small gap between transactions, adjust if needed

    // --- Read register 0x00A: ZERO_POS[3:0] | Z_EDGE | Z_PUL_WID[2:0] ---
    uint8_t tx00A[3] = { 0 };
    tx00A[0] = (MT6835_OP_READ << 4) | ((MT6835_REG_ZERO2 >> 8) & 0x0F);
    tx00A[1] = MT6835_REG_ZERO2 & 0xFF;
    tx00A[2] = 0x00;

    SPI.beginTransaction(m_settings);
    digitalWrite(config_data.spi_cs, LOW);
    SPI.transfer(tx00A, 3);
    digitalWrite(config_data.spi_cs, HIGH);
    SPI.endTransaction();

    uint8_t reg00A = tx00A[2];
    uint8_t zero_pos_low = (reg00A >> 4) & 0x0F; // ZERO_POS[3:0]

    uint16_t zero_pos_raw = (static_cast<uint16_t>(zero_pos_high) << 4) | zero_pos_low; // 12-bit value, 0-4095

    Serial.printf("Pin: %u, ZERO_POS raw = 0x%03X (%u), degrees = %.3f\n",
                  config_data.spi_cs, zero_pos_raw, zero_pos_raw,
                  zero_pos_raw * (360.0f / 4096.0f));

    return zero_pos_raw * (360.0f / 4096.0f);
}

void BuffEncoder::send_to_comms() const {
    if (!m_has_valid_read) {
        return;
    }

    Comms::Sendable<BuffEncoderData> sendable;
    sendable.data = comms_data;
    sendable.send_to_comms();
}

void BuffEncoder::print() const{
    Serial.printf("Buff Encoder:\n\t");
    Serial.println(get_angle());
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
