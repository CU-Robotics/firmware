#pragma once

#include "sensors/sensor.hpp"
#include "comms/data/limit_switch_data.hpp"
// tie C pin to 3.3v
// NO pin is the pin to read, it is set up as input pulldown 
// NC should be untied

/// @brief Class to manage a limit switch
class LimitSwitch : public Sensor {
public:
    /// @brief Constructor for the LimitSwitch class
    /// @param config The configuration data for the limit switch
    LimitSwitch(const Cfg::LimitSwitch& config) : Sensor(), config(config), comms_data(config.switch_name) {}

    /// @brief initialize sensor
    void init() override;
    /// @brief Read the state of the limit switch
    void read() override;
    /// @brief Send the limit switch state to comms
    void send_to_comms() const override;
    /// @brief Get whether the limit switch is currently pressed
    /// @return true if the limit switch is pressed, false otherwise
    inline bool get_is_pressed() const { return is_pressed; }
    
private:
    /// @brief Configuration data for the limit switch
    const Cfg::LimitSwitch& config;
    /// @brief Data to be sent to comms
    LimitSwitchData comms_data;

    /// @brief Whether the limit switch is currently pressed
    uint8_t is_pressed = 0;
};
