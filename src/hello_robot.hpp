#pragma once
#include <Arduino.h>
#include <optional>

#include "can_manager.hpp"
#include "comms/comms_layer.hpp"
#include "controls/state.hpp"
#include "controls/reference_governor.hpp"
#include "git_info.h"

#include "safety.hpp"
#include "sensors/buff_encoder.hpp"
#include "state.hpp"
#include "utils/profiler.hpp"
#include "utils/boot_splash.hpp"


#include "sensors/transmitter/transmitter_utils.hpp"
#include "sensors/transmitter/transmitter_manager.hpp"
#include "sensors/d200.hpp"

#include "controls/controller_manager.hpp"
#include "controls/estimator_manager.hpp"
#include "sensors/StereoCamTrigger.hpp"
#include "sensors/RefSystem.hpp"
#include "utils/profiler.hpp"

#include "sensor_manager.hpp"
#include <TeensyDebug.h>


#include "comms/data/hive_data.hpp"
#include "comms/data/sendable.hpp"
#include "comms/data/robot_state_data.hpp"
#include "utils/timing.hpp"
#include "utils/watchdog.hpp"
extern "C" void reset_teensy(void);
// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2
#ifdef PROFILER
Profiler prof;
#endif
class HelloRobot {
private:
    CANManager can;
    //RefSystem ref;
    TransmitterManager transmitter_manager;

    //Comms::CommsLayer comms_layer;
    SensorManager sensor_manager;
	EstimatorManager estimator_manager;
	ControllerManager controller_manager;
	
	Watchdog watchdog;
    bool not_safety_mode = false;
    uint32_t loopc = 0; // Loop counter for heartbeat
	// main loop timers
    Timer loop_timer;
    Timer stall_timer;
    Timer gimbal_power_timer;
	// param to specify whether this is the first loop
    bool is_first_loop = true;

    bool last_gimbal_power = false; // used to detect gimbal power changes
    bool last_loop_slow = false;    // used to detect multiple slow loops in a row
    int slow_loop_counter = 0;      // used to count slow loops in a row
    
    // manual controls variables
    float feed = 0;
    float last_feed = 0;
	// variables for use in loop
	std::optional<Governor> governor;
	std::optional<RobotStateMap> estimated_state_map;
    std::optional<RobotStateMap> reference_map;              
    std::optional<RobotStateMap> target_state_map;// Temp ungoverned state
    std::optional<RobotStateMap> hive_state_map_offset;// Hive offset state
    bool override_request = false;
	
    void setup();
	void crash_report();
    void read_telemetry();
	void process_behaviors();
    void update_controls();
    void check_safety();
	void loop_timing();

public:
    void init();
    void run();
};
