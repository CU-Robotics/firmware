#include "RefSystem.hpp"

RefSystem ref;

uint32_t accumulator = 0;
uint32_t second_accumulator = 0;
uint32_t dt = 0;
uint32_t curr_time = 0;

uint8_t length = 112;
uint8_t packet[REF_MAX_PACKET_SIZE] = {
    0xA5, length, 0x00, 0x00, 0x00,
    0x01, 0x03,
    0x01, 0x02, 0x07, 0x00, 0x01, 0x00
};

void setup() {
    Serial.begin(9800);
    Serial.println("Start");
    ref.init();
    Serial.println("Init");
    curr_time = micros();

    for (int i = 0; i < length; i++) {
        packet[13 + i] = i + 1;
    }
}

void loop() {
    dt = micros() - curr_time;
    curr_time = micros();
    accumulator += dt;
    second_accumulator += dt;

    if (second_accumulator > 1000000u) {
        ref.bytes_sent = 0;
        second_accumulator = 0;
    }

    if (accumulator > REF_MAX_PACKET_DELAY) {
        accumulator = 0;
        // ref.write(packet, length + 5 + 2 + 2 + 6);
    }

    ref.read();

    // Serial.printf("Failed: %u\n", ref.packets_failed);

    Serial.printf("S: %u\tR: %u\tM: %u\t%.2f%%\tF: %u\n", ref.packets_sent, ref.packets_received, ref.packets_sent - ref.packets_received, (1.0 - (double)ref.packets_received / (double)ref.packets_sent) * 100.0, ref.packets_failed);
}