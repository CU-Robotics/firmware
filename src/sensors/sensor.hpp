#pragma once
#include "comms/config_data/sensor.hpp"




/// @brief Abstract class representing a sensor. All sensors should inherit from this class.
class Sensor {
public: 
/// @brief Default constructor
Sensor() = default;
	
/// @brief Initialize the sensor; typically involves setting up the communication link.
virtual void init() = 0;

/// @brief Read data from the sensor and update internal state accordingly.
virtual void read() = 0;
	
/// @brief Initiates a non-blocking hardware read (The "Kick"). Default does nothing.
virtual void request_read() {}
	
/// @brief Prints a formatted dashboard of live sensor values. Default does nothing.
virtual void print_live_data() {}
	
/// @brief Send the current sensor data to the comms layer.
virtual void send_to_comms() const = 0;

};
