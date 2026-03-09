#pragma once
#include "controls/robot_state_map.hpp"
#include "controls/reference_governor.hpp"
/// @brief A unifying interface for all transmitters
class Transmitter {
public:

	Transmitter() = default;
	/// @brief standard destructor
	virtual ~Transmitter() {};
	
	/// @brief Reads raw input
	virtual void read() = 0;
	
	/// @brief initalizes serial connection
	virtual void init() = 0;
	
	/// @brief prints all output values
	virtual void print() = 0;
	virtual void print_raw() = 0;
	
	virtual void send_to_comms() = 0;

	virtual bool is_safety_mode() = 0;

	virtual bool is_hive_mode() = 0;
	virtual bool is_teensy_mode() = 0;

	virtual void manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, Governor& governor, bool not_safety_mode, float& feed, float& last_feed, bool& hive_toggle, bool& safety_toggle) = 0;
};
