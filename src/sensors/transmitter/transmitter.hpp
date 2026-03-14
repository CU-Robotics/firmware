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
	/// @brief prints raw packet values for debugging
	virtual void print_raw() = 0;
	/// @brief sends data to comms
	virtual void send_to_comms() = 0;
	/// @brief Whether the transmitter is currently in safety mode.
	/// @return true if the transmitter is in safety mode, false otherwise.
	virtual bool is_safety_mode() = 0;

	/// @brief Whether the transmitter is currently in hive mode.
	/// @return true if the transmitter is in hive mode, false otherwise.
	virtual bool is_hive_mode() = 0;
	/// @brief Whether the transmitter is currently in teensy mode.
	/// @return true if the transmitter is in teensy mode, false otherwise.
	virtual bool is_teensy_mode() = 0;
	
	/// @brief Uses the transmitter input to update the target state map with the desired setpoints for each state.
	/// @param estimated_state_map The current estimated state of the robot.
	/// @param target_state_map The map of target states to update.
	/// @param governor A reference to the governor to update the reference map if needed.
	/// @param not_safety_mode Whether we are in safety mode.
	/// @param feed The feed value.
	/// @param last_feed The last feed value.
	/// @param hive_toggle Hive toggle value. Used to keep track of when we toggle into hive mode.
	/// @param safety_toggle The safety toggle value. Used to keep track of when we toggle into safety mode.
	virtual void manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, Governor& governor, bool not_safety_mode, float& feed, float& last_feed, bool& hive_toggle, bool& safety_toggle) = 0;
};
