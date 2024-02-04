#define MT6835_BITORDER MSBFIRST
#define MT6835_OP_ANGLE 0b1010
#define MT6835_REG_ANGLE1 0x003
#define MT6835_CPR 2097152
#include "buff_encoder.hpp"
static SPISettings settings(1000000, MT6835_BITORDER, SPI_MODE3);

BuffEncoder::BuffEncoder(int nCS){
    _nCS = nCS;
}

void BuffEncoder::read(){
    uint8_t data[6] = { 0 }; // transact 48 bits
    data[0] = (MT6835_OP_ANGLE << 4);
    data[1] = MT6835_REG_ANGLE1;
    SPI.beginTransaction(settings);
    digitalWrite(_nCS, LOW);
    SPI.transfer(data, 6);
    digitalWrite(_nCS, HIGH);
    SPI.endTransaction();
    int raw_angle = (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
    _radians = raw_angle / (float)MT6835_CPR * (3.14159265 * 2.0);
    // Serial.printf("nCS: %d      %d raw      %0.8f degrees      %0.8f radians\n", nCS, raw_angle, degrees, radians);
    return;
}

float BuffEncoder::get_radians(){
    return _radians;
}