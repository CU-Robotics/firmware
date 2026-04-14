#include "rev_encoder.hpp"
#include <cmath>
#include "comms/data/rev_encoder_data.hpp"
#include "comms/data/sendable.hpp"

void RevEncoder::init() {
    pinMode(this->config.digital_pin, INPUT);  // Set the pin used to measure the encoder to be an input
    freq.begin(this->config.digital_pin, FREQMEASUREMULTI_MARK_ONLY);
    Serial.printf("Rev Encoder %u init: calculating starting value...\n", static_cast<unsigned int>(this->config.encoder_name));
    if(this->config.is_relative){
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

    comms_data.radians = this->radians;
    comms_data.ticks = this->ticks;
}

void RevEncoder::send_to_comms() const {
    Comms::Sendable<RevSensorData> sendable;
        
    sendable.data = comms_data;
    sendable.send_to_comms();
}

float RevEncoder::get_angle_ticks() {
    return this->ticks;
}

float RevEncoder::get_angle_radians() {
    return this->radians - starting_value;
}

void RevEncoder::print() {
    logger.println(LogDestination::Serial, "Rev Encoder:");
    logger.print("\tTicks: ");
    logger.println(ticks);
    logger.print("\tRadians: ");
    logger.println(radians);
}