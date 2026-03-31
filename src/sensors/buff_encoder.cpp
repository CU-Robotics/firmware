#include "buff_encoder.hpp"
#include "comms/data/sendable.hpp"

const SPISettings BuffEncoder::m_settings = SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);

void BuffEncoder::init() {
    // set the SPI pins to the correct mode
    pinMode(config_data.spi_cs, OUTPUT);
    digitalWrite(config_data.spi_cs, HIGH); // set CS high to start
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

    // convert received angle into radians
    int raw_angle = (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
    float radians = raw_angle / (float)MT6835_CPR * (3.14159265 * 2.0);

    // assign angle
    m_angle = radians;

    // Serial.printf("Buff Encoder %u - angle: %f\n", static_cast<uint32_t>(config_data.encoder_name), m_angle);

    comms_data.m_angle = m_angle;
}

void BuffEncoder::send_to_comms() const {
    Comms::Sendable<BuffEncoderData> sendable;
    sendable.data = comms_data;
    sendable.send_to_comms();
}

void BuffEncoder::print() const{
    Serial.printf("Buff Encoder:\n\t");
    Serial.println(get_angle());
}