#pragma once
#include <Arduino.h>
#include <optional>

#include "controls/robot_state_array.hpp"
#include "sensors/transmitter/transmitter_manager.hpp"
#include "sensors/sensor_manager.hpp"
#include "controls/robot_state_array.hpp"
#include "utils/system_log.hpp"
#include "utils/profiler.hpp"
/* Adding new commands is simple.
   add command to command dictionary block below
   If it is live add to live mode enum and switch statement above
   as well as to view_dict lookup table in cmd_live function
 */
#ifdef PROFILER
extern Profiler prof; 
#endif
/// @brief Command line interface for real-time monitoring and debugging over serial.
class RobotCLI {
public:
	/// @brief Collection of Live viewmodes
	enum class LiveMode {
		NONE,
		PROFILE_VIEW,
		TRANSMITTER,
		ESTIMATED_STATE,
		TARGET_STATE,
		SENSORS,
		HEARTBEAT
	};
	/// @brief number of live views allowed at once
	static constexpr uint8_t MAX_LIVE_VIEWS = 4;
	/// @brief size of CLI Buffer
    static constexpr size_t CLI_BUFFER_SIZE = 64;
    /// @brief Links CLI commands to active robot states and managers and binds the system log buffer.
    /// @param tx Reference to transmitter manager
    /// @param sensors Reference to sensor manager
    /// @param estimated_state Reference to current estimated robot state map
    /// @param target_state Reference to current target robot state map
    /// @param loop_counter Reference to loop counter
    void init(
        TransmitterManager& tx,
        SensorManager& sensors,
        const RobotStateArray& estimated_state,
        const RobotStateArray& target_state,
        const uint32_t& loop_counter
    );
	/// @brief check for serial input and redraws the active dashboard.
    void process();

private:
    // Subsystem handles
    /// @brief Pointer to transmitter manager handle
    TransmitterManager* transmitter_manager = nullptr;
    /// @brief Pointer to sensor manager handle
    SensorManager* sensor_manager = nullptr;
    /// @brief Pointer to estimated robot state map
    const RobotStateArray* estimated_state_array = nullptr;
    /// @brief Pointer to target robot state map
    const RobotStateArray* target_state_array = nullptr;
    /// @brief Pointer to loop counter
    const uint32_t* loopc = nullptr;
    
    /// @brief array of current live views
    LiveMode active_views[MAX_LIVE_VIEWS];
    /// @brief number of active live views
    uint8_t num_active_views = 0;
    /// @brief time since the live view was refreshed
    uint32_t last_redraw_time = 0;
    /// @brief refresh rate in milliseconds
    uint32_t redraw_interval = 1000; 
	/// @brief CLI Buffer
    char cli_buffer[CLI_BUFFER_SIZE] = {0};
	/// @brief index for cli_buffer
    uint8_t cli_index = 0;
	/// @brief flag for live CLI printing
    bool live_profiler_active = false;
    
    /// @brief CLI ping function
    void cmd_ping();
    /// @brief CLI help function
    void cmd_help();
    /// @brief CLI live view function
    void cmd_live();
    /// @brief CLI function to handle logging
    void cmd_log();
    /// @brief handles case where live command is used
    void render_live_view();
    /// @brief Parse CLI stream from serial
    void parse_serial_stream();
    
};
