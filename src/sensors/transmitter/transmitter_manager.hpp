#pragma once

#include "comms/config_data/transmitter.hpp"
#include "sensors/transmitter/transmitter.hpp"
#include <memory>
/// @brief Manage the configured transmitter
class TransmitterManager {
public:
    /// @brief Default constructor
    TransmitterManager() {};
    /// @brief Default destructor
    ~TransmitterManager() {};
    /// @brief Initializes the transmitter manager by creating a new transmitter object based on the provided configuration.
    /// @param transmitter_config The configuration for the transmitter, including its type and any specific configuration data for that type.
    void init(const Cfg::Transmitter& transmitter_config);
    /// @brief Reads data from the transmitter
    void read();
    /// @brief Sends data from the transmitter to comms
    void send_to_comms();
    /// @brief Whether the transmitter is currently in safety mode.
    /// @return true if the transmitter is in safety mode, false otherwise.
    bool is_safety_mode();
    /// @brief Whether the transmitter is currently in hive mode.
    /// @return true if the transmitter is in hive mode, false otherwise.
    bool is_hive_mode();
    /// @brief Whether the transmitter is currently in teensy mode.
    /// @return true if the transmitter is in teensy mode, false otherwise.
    bool is_teensy_mode();

    bool mode_changed();
    
    /// @copydoc Transmitter::manual_controls
    void manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, bool not_safety_mode, float& feed, float& last_feed);

private:
    /// @brief pointer to the transmitter object. This is allocated and assigned upin initialization based on the provided configuration.
    std::unique_ptr<Transmitter> transmitter;


};