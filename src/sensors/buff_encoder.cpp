#include "buff_encoder.hpp"

const SPISettings BuffEncoder::m_settings = SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);

bool BuffEncoder::read() {

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

    // assign angle
    m_angle = radians;

    //add angle to the data struct
    buff_sensor_data.m_angle = m_angle;
    return true;
}


void BuffEncoder::print() {
    Serial.printf("Buff Encoder:\n\t");
    Serial.println(m_angle);
}