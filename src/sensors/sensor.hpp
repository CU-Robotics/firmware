#pragma once
#include "comms/config_data/sensor.hpp"
#include "robot_state_map.hpp"


/// @brief Abstract class representing a sensor. All sensors should inherit from this class.
class Sensor {
public: 
/// @brief Default constructor
Sensor() = default;
/// @brief Initialize the sensor; typically involves setting up the communication link.
virtual void init() = 0;

/// @brief Read data from the sensor and update internal state accordingly.
virtual void read() = 0;

/// @brief Bind local state map with estimated state map
/// @param Bind local state map with global estimated state map	
virtual void provide_isr_map(std::optional<RobotStateMap> *map) {}

/// @brief Send the current sensor data to the comms layer.
virtual void send_to_comms() const = 0;

/// @brief Prints a formatted dashboard of live sensor values. Default does nothing.
virtual void print_live_data(){}
};
