#include "transmitter_manager.hpp"
#include "sensors/transmitter/ET16S.hpp"
#include "sensors/transmitter/dr16.hpp"

void TransmitterManager::init(const Cfg::Transmitter& transmitter_config) {
    Serial.printf("Initializing TransmitterManager with transmitter type: %d\n", static_cast<int>(transmitter_config.transmitter_type));
    switch (transmitter_config.transmitter_type) {
        case Cfg::TransmitterType::DR16:
            transmitter = std::make_unique<DR16>(transmitter_config.dr16);
            break;
        case Cfg::TransmitterType::ET16S:
            transmitter = std::make_unique<ET16S>(transmitter_config.et16s);
            break;
    }

    transmitter->init();

    Serial.printf("Transmitter initialized with type: %d\n", static_cast<int>(transmitter_config.transmitter_type));
}

void TransmitterManager::read() {
    if (transmitter) {
        transmitter->read();
    } else {
        safety::safety_procedure("TransmitterManager::read called before transmitter was initialized");
    }
}

void TransmitterManager::print_live_data() {
    if (transmitter) {
        transmitter->print_live_data();
    } else {
		safety::safety_procedure("TransmitterManager::print_live_data called before transmitter was initialized");
    }
}

void TransmitterManager::send_to_comms() {
    if (transmitter) {
        transmitter->send_to_comms();
    } else {
        safety::safety_procedure("TransmitterManager::send_to_comms called before transmitter was initialized");
    }
}

bool TransmitterManager::is_safety_mode() {
    if (transmitter) {
        return transmitter->is_safety_mode();
    } else {
        safety::safety_procedure("TransmitterManager::is_safety_mode called before transmitter was initialized");
        return true; // default to safety mode if not initialized
    }
}

bool TransmitterManager::is_hive_mode() {
    if (transmitter) {
        return transmitter->is_hive_mode();
    } else {
        safety::safety_procedure("TransmitterManager::is_hive_mode called before transmitter was initialized");
        return false; // default to not hive mode if not initialized
    }
}

bool TransmitterManager::is_teensy_mode() {
    if (transmitter) {
        return transmitter->is_teensy_mode();
    } else {
        safety::safety_procedure("TransmitterManager::is_teensy_mode called before transmitter was initialized");
        return false; // default to not teensy mode if not initialized
    }
}

bool TransmitterManager::mode_changed() {
    if (transmitter) {
        return transmitter->mode_changed();
    } else {
        safety::safety_procedure("TransmitterManager::mode_changed called before transmitter was initialized");
        return false; // default to no mode change if not initialized
    }
}

void TransmitterManager::manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, bool not_safety_mode, float& feed, float& last_feed) {
    if (transmitter) {
        transmitter->manual_controls(estimated_state_map, target_state_map, not_safety_mode, feed, last_feed);
    } else {
        safety::safety_procedure("TransmitterManager::manual_controls called before transmitter was initialized");
    }
}
