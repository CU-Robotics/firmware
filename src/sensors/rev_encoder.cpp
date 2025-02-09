#include "rev_encoder.hpp"

 RevEncoder::RevEncoder(uint8_t encoder_pin) : Sensor(SensorType::REVENC) {
		this->in_pin = encoder_pin;
		pinMode(this->in_pin, INPUT);  // Set the pin used to measure the encoder to be an input
		freq.begin(this->in_pin, FREQMEASUREMULTI_MARK_ONLY);
	};

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

bool RevEncoder::read() {
    while (this->freq.available() > 1) {
        this->freq.read();
    }

    if (freq.available()) {
        int frequency = round(this->freq.countToNanoseconds(this->freq.read()) / 1000);
        this->ticks = frequency % 1024;
        this->radians = (((float)this->ticks) / 1024.0) * M_PI * 2;
    }
    //copy the data to the data struct
    rev_sensor_data.id = id_;
    rev_sensor_data.ticks = ticks;
    rev_sensor_data.radians = radians;
    return true;
    
}

float RevEncoder::get_angle_ticks() {
    return this->ticks;
}

float RevEncoder::get_angle_radians() {
    return (this->radians-starting_value);
}

void RevEncoder::print() {
    Serial.println("Rev Encoder:");
    Serial.print("\tTicks: ");
    Serial.println(ticks);
    Serial.print("\tRadians: ");
    Serial.println(radians);
}