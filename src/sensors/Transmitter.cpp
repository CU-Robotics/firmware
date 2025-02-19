#include "Transmitter.hpp"

TransmitterType Transmitter::who_am_i() {
	uint16_t ET16S_PACKET_SIZE = 25; //Change this
	uint8_t raw_input[ET16S_PACKET_SIZE] = { 0 };
	
	//Serial Initilization
	Serial8.begin(100000, SERIAL_8E1_RXINV_TXINV);
	Serial8.flush();
	Serial8.clear();

	int counter = 0; //How many sequential elements have been found
	
	uint32_t start_time = millis();
	uint32_t timeout = 100; // Times out after 100 microseconds
	while(counter != 5){
		raw_input[24] = 0;
		if (Serial8.available() < (2*ET16S_PACKET_SIZE)) {
			if ((millis() - start_time) < timeout) { return TransmitterType::DR16; }
			continue;
		}
		// We read until we find the start byte of the new packet (0x0f)
		// this ensures we read one packet per loop 

		while ((Serial8.peek() != 0x0f) && ((millis() - start_time) < timeout)) {
			Serial8.read();
		}
		// Fill raw input array
		for (int i = 0; i < ET16S_PACKET_SIZE+1;i++) {
			raw_input[i] = Serial8.read();
		}
		if (raw_input[0] == 0x0f) {
			counter += 1;
		}
		else {
			Serial.println();
			Serial.print("found dr16");
			Serial.println();
			return TransmitterType::DR16;
		}
	}
	Serial.print("found ET16S");
	return TransmitterType::ET16S;
}
