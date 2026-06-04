#pragma once
#include <Arduino.h>
#include <optional>

#include "can_manager.hpp"
#include "comms/comms_layer.hpp"
#include "controls/reference_governor.hpp"
#include "controls/state.hpp"
#include "git_info.h"

#include "robot_state_map.hpp"
#include "safety.hpp"
#include "sensors/buff_encoder.hpp"
#include "state.hpp"
#include "utils/boot_splash.hpp"
#include "utils/profiler.hpp"

#include "sensors/d200.hpp"
#include "sensors/transmitter/transmitter_manager.hpp"
#include "sensors/transmitter/transmitter_utils.hpp"

#include "controls/controller_manager.hpp"
#include "controls/estimator_manager.hpp"
#include "sensors/RefSystem.hpp"
#include "sensors/StereoCamTrigger.hpp"
#include "utils/profiler.hpp"

#include "sensor_manager.hpp"
#include <TeensyDebug.h>
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

    /// @brief Hardware watchdog that resets the Teensy if the loop hangs.
    Watchdog watchdog;

    // ==========================================
    // SYSTEM TIMERS & COUNTERS
    // ==========================================

    /// @brief Timer used to strictly regulate the loop to LOOP_FREQ.
    Timer loop_timer;

    /// @brief Timer used to detect stall conditions and compute delta-time (dt).
    Timer stall_timer;

    /// @brief Timer to track how long gimbal power has been active.
    Timer gimbal_power_timer;

    /// @brief Absolute count of executed loops since boot. Used for heartbeat math.
    uint32_t loopc = 0;

    /// @brief Counts consecutive slow loops to trigger a hard reset if the system locks.
    int slow_loop_counter = 0;

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
    bool not_safety_mode = false;

    /// @brief Param to specify whether this is the first loop.
    bool is_first_loop = true;

    /// @brief Cache of the previous loop's gimbal power state to detect changes.
    bool last_gimbal_power = false;

    /// @brief Used to detect multiple slow loops in a row
    bool last_loop_slow = false;

    /// @brief Whether the active robot config contains the lower feeder state.
    bool has_lower_feeder = false;

    // ==========================================
    // STATE MAPS
    // ==========================================

    /// @brief Governor
    std::optional<Governor> governor;

    /// @brief Map containing the current estimated state of the robot.
    std::optional<RobotStateMap> estimated_state_map;
    
    /// @brief Interrupt safe estimated state map
	std::optional<RobotStateMap> estimated_state_map_interrupt_safe; 

    /// @brief Map containing the immediate reference values handed to controllers.
    std::optional<RobotStateMap> reference_map;

    /// @brief Temp ungoverned state
    std::optional<RobotStateMap> target_state_map;

    /// @brief Hive offset state
    std::optional<RobotStateMap> hive_state_map_offset;

    /// @brief check to see if there is a crash report, and if so, print it repeatedly
    void crash_report();

    /// @brief Reads data from CAN, RefSystem, Transmitter, and Sensors.
    void read_telemetry();

    /// @brief Processes manual inputs, hive modes, and state overrides.
    void process_behaviors();

    /// @brief Steps estimators, governors, and controllers to generate motor targets.
    void update_controls();

    /// @brief Checks loop timing/safety constraints and writes to the CAN bus.
    void check_safety();

    /// @brief LED hearbeat, feeds the watchdog, and ensures consistent loop time.
    void loop_timing();

  public:
    /**
     * @brief Bootstraps the robot's architecture.
     * * Downloads the active configuration from the Hive data layer and uses it
     * to instantiate the state maps, reference governor, and hardware managers.
     */
    void init();

    /// @brief Begins the main loop.
    void run();
};
