#pragma once

#include "sensors/sensor.hpp"
#include "comms/data/limit_switch_data.hpp"
// tie C pin to 3.3v
// NO pin is the pin to read, it is set up as input pulldown 
// NC should be untied

/// @brief Class to manage a limit switch
class LimitSwitch : Sensor {
public:
    /// @brief Constructor for the LimitSwitch class
    /// @param config The configuration data for the limit switch
    LimitSwitch(const Cfg::LimitSwitch& config) : Sensor(), config(config), comms_data(config.switch_name) {}

    void init() override;

    void read() override;

    void send_to_comms() const override;

    inline bool get_is_pressed() const { return is_pressed; }
    
private:
    const Cfg::LimitSwitch& config;
    LimitSwitchData comms_data;

    uint8_t is_pressed = 0;
};
