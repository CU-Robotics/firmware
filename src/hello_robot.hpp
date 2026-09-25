#pragma once
#include <Arduino.h>
#include <optional>

#include "sensors/can/can_manager.hpp"
#include "comms/comms_layer.hpp"
#include "controls/reference_governor.hpp"
#include "controls/state.hpp"
#include "git_info.h"

#include "controls/robot_state_array.hpp"
#include "utils/safety.hpp"
#include "utils/safety_state.hpp"
#include "sensors/buff_encoder.hpp"
#include "comms/config_data/state.hpp"
#include "utils/boot_splash.hpp"

#include "sensors/d200.hpp"
#include "sensors/transmitter/transmitter_manager.hpp"
#include "sensors/transmitter/transmitter_utils.hpp"

#include "controls/controller_manager.hpp"
#include "controls/estimator_manager.hpp"
#include "sensors/RefSystem.hpp"

#include "sensors/StereoCamTrigger.hpp"

#include "sensors/sensor_manager.hpp"
#include <TeensyDebug.h>
#include "utils/profiler.hpp"
#include "utils/system_log.hpp"
#include "utils/robot_cli.hpp"
#include <wiring.h>

#include "comms/data/hive_data.hpp"
#include "comms/data/robot_state_data.hpp"
#include "comms/data/sendable.hpp"
#include "utils/timing.hpp"
#include "utils/watchdog.hpp"

extern "C" void reset_teensy(void);

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Safety constants
/// @brief A loop longer than this (twice the nominal period) is considered slow and disarms the motors.
constexpr float SLOW_LOOP_THRESHOLD_S = 2.0f / LOOP_FREQ;
/// @brief Consecutive slow loops tolerated before the Teensy is reset.
constexpr int MAX_CONSECUTIVE_SLOW_LOOPS = 11;
/// @brief When disarmed, a feeder position whose fractional part exceeds this is rounded up to the next ball.
constexpr float FEED_ROUND_UP_FRACTION = 0.2f;

#ifdef PROFILER
extern Profiler prof; 
#endif

/// @brief Coordinates all hardware, networking, and control systems.
class HelloRobot {
  private:
    // ==========================================
    // MANAGERS & HARDWARE INTERFACES
    // ==========================================

    /// @brief Manages all CAN bus read/write operations and motor command queues.
    CANManager can;

    /// @brief Manages the ET16S/DR16 radio transmitters
    TransmitterManager transmitter_manager;

    /// @brief Handles initialization and polling for all connected I2C/SPI sensors.
    SensorManager sensor_manager;

    /// @brief Steps state estimators for robot kinematics.
    EstimatorManager estimator_manager;

    /// @brief Calculates controls and feed-forward outputs for all physical actuators.
    ControllerManager controller_manager;
    
    /// @brief Manages the command line interface
    RobotCLI cli;
    
    /// @brief Hardware watchdog that resets the Teensy if the loop hangs.
    Watchdog watchdog;

    // ==========================================
    // SYSTEM TIMERS & COUNTERS
    // ==========================================

    /// @brief Timer used to strictly regulate the loop to LOOP_FREQ.
    Timer loop_timer;

    /// @brief Timer used to detect stall conditions and compute delta-time (dt).
    Timer stall_timer;

    /// @brief Absolute count of executed loops since boot. Used for heartbeat math.
    uint32_t loopc = 0;

    /// @brief Counts consecutive slow loops to trigger a hard reset if the system locks.
    int consecutive_slow_loops = 0;

    // ==========================================
    // ROBOT VARIABLES
    // ==========================================

    /// @brief Target position for the feeder mechanism.
    float feed = 0;

    /// @brief Previous target position for the feeder mechanism.
    float last_feed = 0;

    /// @brief Flag set when Hive requests an override.
    bool override_request = false;

    // ==========================================
    // STATE FLAGS
    // ==========================================

    /// @brief Flag indicating if the motors are armed and allowed to move.
    bool motors_armed = false;

    /// @brief Param to specify whether this is the first loop.
    bool is_first_loop = true;

    /// @brief Whether the active robot config contains the lower feeder state.
    bool has_lower_feeder = false;

    // ==========================================
    // STATE ARRAYS
    // ==========================================

    /// @brief Governor
    std::optional<Governor> governor;

    /// @brief Array containing the current estimated state of the robot.
    std::optional<RobotStateArray> estimated_state_array;
    
    /// @brief Interrupt safe estimated state array
    std::unique_ptr<RobotStateArray> estimated_state_array_interrupt_safe;

    /// @brief Array containing the immediate reference values handed to controllers.
    std::optional<RobotStateArray> reference_array;

    /// @brief Temp ungoverned state
    std::optional<RobotStateArray> target_state_array;

    /// @brief Hive offset state
    std::optional<RobotStateArray> hive_state_array_offset;
    
	// ==========================================
    // Major Loop functions
    // ==========================================
	/// @brief check to see if there is a crash report, and if so, print it repeatedly
	void crash_report();
	
	/// @brief Reads data from CAN, RefSystem, Transmitter, and Sensors.
    void read_telemetry();
	
	/// @brief Processes manual inputs, hive modes, and state overrides.
	void process_behaviors();
	
	/// @brief Steps estimators, governors, and controllers to generate motor targets.
    void update_controls();

    /// @brief Handles all comms data transfers
    void update_comms();
	
	/// @brief Checks loop timing/safety constraints and writes to the CAN bus.
    void check_safety();
    
    /// @brief Measures loop time and resets the Teensy after too many consecutive slow loops.
    /// @param loop_dt Set to the measured loop time in seconds
    /// @return true if this loop was slow
    /// @note Reporting is left to check_safety so logging can't delay disarming the motors.
    bool check_slow_loop(float& loop_dt);

    /// @brief Holds the feeders at their current position so they don't jump when re-armed.
    void hold_feeder_position();
	
	/// @brief LED hearbeat, feeds the watchdog, and ensures consistent loop time.
	void loop_timing();


public:
    /**
     * @brief Bootstraps the robot's architecture.
     * * Downloads the active configuration from the Hive data layer and uses it
     * to instantiate the state arrays, reference governor, and hardware managers.
     */
    void init();

    /// @brief Begins the main loop.
    void run();
};
