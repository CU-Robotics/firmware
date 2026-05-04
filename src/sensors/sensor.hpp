#pragma once
#include "comms/config_data/sensor.hpp"

/// @brief CRTP Base class representing a sensor. 
template <typename Derived>
class Sensor {
public: 
	/// @brief Default constructor
	Sensor() = default;
	/// @brief Initialize the sensor; typically involves setting up the communication link.
	void init() {
        static_cast<Derived*>(this)->init_impl();
    }
	/// @brief Read data from the sensor and update internal state accordingly.
	void read() {
        static_cast<Derived*>(this)->read_impl();
    }
	/// @brief Prints a formatted dashboard of live sensor values. Default does nothing.
	void print_live_data() {
        static_cast<Derived*>(this)->print_live_data_impl();
    }
	/// @brief Send the current sensor data to the comms layer.
	void send_to_comms() const {
        static_cast<const Derived*>(this)->send_to_comms_impl();
    }
protected:
	/// @brief blank implementation to make live printing optional
	void print_live_data_impl() {}
};
