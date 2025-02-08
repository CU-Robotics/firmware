#include "buff_encoder.hpp"

const SPISettings BuffEncoder::m_settings = SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);

float BuffEncoder::read() {

    uint8_t data[6] = { 0 }; // transact 48 bits

    // set the operation
    data[0] = (MT6835_OP_ANGLE << 4);
    data[1] = MT6835_REG_ANGLE1;

    // do the SPI transfer
    SPI.beginTransaction(m_settings);
    digitalWrite(m_CS, LOW);
    SPI.transfer(data, 6);
    digitalWrite(m_CS, HIGH);
    SPI.endTransaction();


    // convert received angle into radians
    int raw_angle = (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
    float radians = raw_angle / (float)MT6835_CPR * (3.14159265 * 2.0);

    // assign and return angle
    m_angle = radians;
    return radians;
}

void BuffEncoder::serialize(uint8_t* buffer, size_t& offset) {
    buffer[offset++] = id_;
    memcpy(buffer + offset, &m_angle, sizeof(m_angle));
    offset += sizeof(m_angle);
}

void BuffEncoder::print() {
    Serial.printf("Buff Encoder:\n\t");
    Serial.println(m_angle);
}