#pragma once

#include "comms/config_data/transmitter.hpp"
#include "sensors/transmitter/transmitter.hpp"
#include <memory>

class TransmitterManager {
public:
    TransmitterManager() {}
    ~TransmitterManager() {};

    void init(const Cfg::Transmitter& transmitter_config);

    void read();

    void send_to_comms();

    bool is_safety_mode();
    bool is_hive_mode();
    bool is_teensy_mode();

    void manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, Governor& governor, bool not_safety_mode, float& feed, float& last_feed, bool& hive_toggle, bool& safety_toggle);

private:
    std::unique_ptr<Transmitter> transmitter;
};