#include "buff_encoder.hpp"
#include "comms/data/sendable.hpp"
#include "safety.hpp"
#include <SPI.h>

const SPISettings BuffEncoder::m_settings = SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);

void BuffEncoder::init() {
    // set the SPI pins to the correct mode
    pinMode(config_data.spi_cs, OUTPUT);
    digitalWrite(config_data.spi_cs, HIGH); // set CS high to start
}
void BuffEncoder::isr_start_transfer(EventResponderRef spi_event) {
	tx_buffer[0] = (MT6835_OP_ANGLE << 4);
	tx_buffer[1] = MT6835_REG_ANGLE1;
	SPI1.beginTransaction(m_settings);
	digitalWrite(config_data.spi_cs, LOW);

	SPI1.transfer(tx_buffer, rx_buffer, sizeof(tx_buffer), spi_event); //after testing make this an assert_or_safety_procedure()
}
void BuffEncoder::isr_stop_transfer(EventResponderRef spi_event) {
	digitalWrite(config_data.spi_cs, HIGH);
    SPI1.endTransaction();
    arm_dcache_delete(rx_buffer, 32);
	
}
void BuffEncoder::read() {
		// Serial.printf("Pin: %u, Sending Buff Encoder read command\n", config_data.spi_cs);

		// convert received angle into radians
		int raw_angle = (rx_buffer[2] << 13) | (rx_buffer[3] << 5) | (rx_buffer[4] >> 3);
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
void BuffEncoder::print_live_data() {
    // Note: casting get_name() to int so it prints the enum number
    Serial.printf(" [Buff Encoder %d] Angle (rad): %8.4f\n", 
                  (int)get_name(), get_angle());
}
