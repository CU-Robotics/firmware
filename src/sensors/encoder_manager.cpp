#include "encoder_manager.hpp"

EncoderManager* EncoderManager::instance = nullptr;

void EncoderManager::init() {
	spi_event.attachInterrupt(encoder_isr_wrapper);
}

void EncoderManager::request_read() {
}

void EncoderManager::read() {
}
void EncoderManager::encoder_isr() {
    // Tell the active encoder to clean up its own private variables
    encoders[current_index]->read();

    current_index = current_index + 1;

    // Tell the next encoder to start using its own private variables
    if (current_index < 4) {
        encoders[current_index]->request_read();
    }
}
void EncoderManager::encoder_isr_wrapper(EventResponderRef spi_event) {
    if (instance != nullptr) {
        // Route the execution back into the specific object instance
        instance->encoder_isr();
    }
}
