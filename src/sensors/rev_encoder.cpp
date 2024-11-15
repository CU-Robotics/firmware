#include "rev_encoder.hpp"


void RevEncoder::init(uint8_t encoder_pin, bool is_relative) {
    this->in_pin = encoder_pin;
    pinMode(this->in_pin, INPUT);  // Set the pin used to measure the encoder to be an input
    freq.begin(this->in_pin, FREQMEASUREMULTI_MARK_ONLY);
    if(is_relative){
        for(int i=0;i<500;i++){
            this->read();
            delayMicroseconds(5);
        }
        starting_value = this->radians;
    }
}

void RevEncoder::read() {
    while (this->freq.available() > 1) {
        this->freq.read();
    }

    if (freq.available()) {
        int frequency = round(this->freq.countToNanoseconds(this->freq.read()) / 1000);
        this->ticks = frequency % 1024;
        this->radians = (((float)this->ticks) / 1024.0) * M_PI * 2;
    }
}

float RevEncoder::get_angle_ticks() {
    return this->ticks;
}

float RevEncoder::get_angle_radians() {
    return (this->radians-starting_value);
}

void RevEncoder::serialize(uint8_t* buffer, size_t& offset) {
    buffer[offset++] = id_;
    memcpy(buffer + offset, &ticks, sizeof(ticks));
    offset += sizeof(ticks);
    memcpy(buffer + offset, &radians, sizeof(radians));
    offset += sizeof(radians);
    //TODO: starting value and freq if needed
    
}

