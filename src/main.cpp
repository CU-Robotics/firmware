#include <Arduino.h>

#include "can_manager.hpp"
#include "comms/comms_layer.hpp"
#include "controls/state.hpp"
#include "controls/reference_governor.hpp"
#include "git_info.h"

#include "safety.hpp"
#include "sensors/buff_encoder.hpp"
#include "state.hpp"
#include "utils/profiler.hpp"


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

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

extern "C" void reset_teensy(void);

// Declare global objects

CANManager can;
RefSystem ref;
TransmitterManager transmitter_manager;

Comms::CommsLayer comms_layer;

#ifdef PROFILER
Profiler prof;
#endif

SensorManager sensor_manager;
EstimatorManager estimator_manager;
ControllerManager controller_manager;

Watchdog watchdog;

// DONT put anything else in this function. It is not a setup function
void print_logo() {
    if (Serial) {
        Serial.println("TEENSY SERIAL START\n\n");
        Serial.print("\033[1;33m");
        Serial.println("                  .:^!?!^.                        ");
        Serial.println("           .:~!?JYYYJ?7?Y5Y7!!.                   ");
        Serial.println("         :?5YJ?!~:.      ^777YP?.                 ");
        Serial.println("         5G~                  ~YP?:               ");
        Serial.println("         7P5555Y:               ^YP?:....         ");
        Serial.println("        ~55J7~^.   ..    .        ^JYYYYYYYYYJJ!. ");
        Serial.println("        YG^     !Y5555J:^PJ    Y5:      ...::^5G^ ");
        Serial.println("       :GY    .YG?^..^~ ~GY    5G^ ^!~~^^^!!~7G?  ");
        Serial.println(" .!JYYY5G!    7BJ       ~GY    5G^ ~??JJJY555GP!  ");
        Serial.println("^55!^:.^~.    ^PP~   .: ^GP:  ^PP:           :7PY.");
        Serial.println("YG^            :JP5YY55: ~YP55PY^              ~GJ");
        Serial.println("?G~      .?7~:   .^~~^.    .^:.                :G5");
        Serial.println(".5P^     7BYJ5YJ7^.                          .~5P^");
        Serial.println(" .JPJ!~!JP?  .:~?PP^            .:.    .^!JYY5Y!. ");
        Serial.println("   :!?!?!:       5P.         .!Y5YYYJ?Y5Y?!^:.    ");
        Serial.println("                 7G7        7GY!. .:~!^.          ");
        Serial.println("                  JG!      :G5                    ");
        Serial.println("                   7PY!^^~?PY:                    ");
        Serial.println("                    .!JJJJ?^                      ");
        Serial.print("\033[0m");
        Serial.println("\n\033[1;92mFW Ver. 2.1.0");
        Serial.printf("\nLast Built: %s at %s", __DATE__, __TIME__);
        Serial.printf("\nGit Hash: %s", GIT_COMMIT_HASH);
        Serial.printf("\nGit Branch: %s", GIT_BRANCH);
        Serial.printf("\nCommit Message: %s", GIT_COMMIT_MSG);
        Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}

// Master loop
int main() {
    uint32_t loopc = 0; // Loop counter for heartbeat

    Serial.begin(115200); // the serial monitor is actually always active (for
                          // debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);

    print_logo();

    // check to see if there is a crash report, and if so, print it repeatedly
    // over Serial in the future, we'll send this directly over comms
    if (CrashReport) {
        while (1) {
            Serial.println(CrashReport);
            Serial.println("\nReflash to clear CrashReport (and also please "
                           "fix why it crashed)");
            delay(1000);
        }
    }

    // Execute setup functions
    pinMode(LED_BUILTIN, OUTPUT);
    
    comms_layer.init();

    // Configure the robot from comms data, which is filled on Hive.
    Serial.println("Configuring...");

    comms_layer.configure();
    
    const Cfg::RobotConfig& config = comms_layer.get_hive_data().config;
    
    Serial.println("Configured!");
    Serial.printf("transmitter type: %d\n", static_cast<int>(config.transmitter.transmitter_type));
    
    // initialize objects
    Governor governor(config.states);
    
    can.init(config.motors);
    
    safety::register_safety_function([&](){ can.issue_safety_mode(); });

    ref.init();
    transmitter_manager.init(config.transmitter);
    
    // initialize sensors
    sensor_manager.init(config);
    
    estimator_manager.init(config.estimators, sensor_manager, can);
    
    // generate controller outputs based on governed references and estimated
    // state
    controller_manager.init(config.controllers, can);
    
    // variables for use in main
    RobotStateMap estimated_state_map(config.states);
    RobotStateMap reference_map(config.states);                   
    RobotStateMap target_state_map(config.states);// Temp ungoverned state
    RobotStateMap hive_state_map_offset(config.states);// Hive offset state
    bool override_request = false;

    // param to specify whether this is the first loop
    int count_one = 0;

    bool has_lower_feeder = false;
    for (const auto& state_config : config.states) {
        if (state_config.name == Cfg::StateName::LowerFeeder) {
            has_lower_feeder = true;
            break;
        }
    }

    bool not_safety_mode = false;
    bool last_gimbal_power = false; // used to detect gimbal power changes
    bool last_loop_slow = false;    // used to detect multiple slow loops in a row
    int slow_loop_counter = 0;      // used to count slow loops in a row
    
    // manual controls variables
    float feed = 0;
    float last_feed = 0;

    // main loop timers
    Timer loop_timer;
    Timer stall_timer;
    Timer gimbal_power_timer;

    // start the main loop watchdog
    watchdog.start();

    Serial.println("Entering main loop...\n");

    // Main loop
    while (true) {
        // start main loop time timer
        stall_timer.start();

        // read CAN and send motor states to comms
        can.read();
        can.send_to_comms();

        // read ref and send to comms
        ref.read();
        ref.send_to_comms();

        // read transmitter and send to comms
        transmitter_manager.read();
        transmitter_manager.send_to_comms();

        // read sensors and send to comms
        // this happens in one function call 
        sensor_manager.read();
        sensor_manager.send_to_comms();


        // print loopc every second to verify it is still alive
        if (loopc % 1000 == 0) {
            Serial.println(loopc);
        }

        // manual controls on firmware
        transmitter_manager.manual_controls(estimated_state_map, target_state_map, not_safety_mode, feed, last_feed);

        // check if we want to use hive controls instead
        if (transmitter_manager.is_hive_mode()) {
            // hid_incoming.get_target_state_map(target_state_map);
            target_state_map.from_comms_packet(comms_layer.get_hive_data().target_state_data.state);
            last_feed = target_state_map[Cfg::StateName::Feeder].get_position();            
        }

        // override temp state if needed. Dont override in teensy mode so the sentry doesnt move during inspection
        if (comms_layer.get_hive_data().override_state_data.active && !(transmitter_manager.is_teensy_mode())) {
            // clear the request
            comms_layer.get_hive_data().override_state_data.active = false;

        Serial.printf("Overriding state with hive state\n");
            hive_state_map_offset.from_comms_packet(comms_layer.get_hive_data().override_state_data.state);

            estimated_state_map = hive_state_map_offset;
            override_request = true;
        }

        // step estimates and construct estimated state
        estimator_manager.step(estimated_state_map, override_request);
        // estimated_state_map.print();

        override_request = false;

        if ((feed - estimated_state_map[Cfg::StateName::Feeder].get_position() > 2 && transmitter_manager.is_teensy_mode()) ||
            (target_state_map[Cfg::StateName::Feeder].get_position() - estimated_state_map[Cfg::StateName::Feeder].get_position() > 2 &&
             transmitter_manager.is_hive_mode())) {
            Serial.printf("Feeder is lowkey jammed. current ball count: %f, feed: %f, hive target: %f\n",
                            estimated_state_map[Cfg::StateName::Feeder].get_position(), feed, target_state_map[Cfg::StateName::Feeder].get_position());
            feed = estimated_state_map[Cfg::StateName::Feeder].get_position() + 1;
            governor.set_position_reference(Cfg::StateName::Feeder, feed);
        }

        // if first loop set target state to estimated state
        if (count_one == 0) {
            governor.set_reference_map(estimated_state_map);
            count_one++;
        }

        if (transmitter_manager.mode_changed()) {
            governor.set_reference_map(estimated_state_map);
        }
        // reference govern
        reference_map = governor.step_reference_map(target_state_map);

        // generate motor outputs from controls
        controller_manager.step(reference_map, estimated_state_map);

        target_state_map.send_to_comms<TargetState>();
        estimated_state_map.send_to_comms<EstimatedState>();

        Comms::Sendable<ConfigurationStatusData> config_status_sendable;
        config_status_sendable.data.is_configured = comms_layer.is_configured() ? 1 : 0;
        config_status_sendable.send_to_comms();

        comms_layer.run();

        bool is_slow_loop = false;

        // check whether this was a slow loop or not

        float dt = stall_timer.delta();
        if (dt > 0.002) {
            // zero the can bus just in case
            can.issue_safety_mode();

            Serial.printf("Slow loop with dt: %f, slow loop count %d\n", dt, slow_loop_counter);
            // mark this as a slow loop to trigger safety mode
            is_slow_loop = true;
            if (last_loop_slow) {
                slow_loop_counter++;
                if (slow_loop_counter > 10) {
                    Serial.printf("Kowabunga bitches\n");
                    reset_teensy();
                }
            } else {
                slow_loop_counter = 0;
            }
        }
        last_loop_slow = is_slow_loop;

        if (!last_gimbal_power && ref.ref_data.robot_performance.gimbal_power_active) {
            gimbal_power_timer.start();
        }
        last_gimbal_power = ref.ref_data.robot_performance.gimbal_power_active;
        bool gimbal_power_recently_turned_on = gimbal_power_timer.get_elapsed_micros_no_restart() < 3000000;

        not_safety_mode =
            (!transmitter_manager.is_safety_mode() &&
             comms_layer.is_configured() && !is_slow_loop && ref.ref_data.robot_performance.gimbal_power_active &&
             !gimbal_power_recently_turned_on);
        
        // Update the shared safety mode state so controllers and estimators can skip checks when in safety mode
        safety::set_safety_mode(!not_safety_mode);
        
        //  SAFETY MODE
        if (not_safety_mode) {
            // SAFETY OFF
            can.write();
            // Serial.printf("Can write\n");
        } else {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.issue_safety_mode();
            governor.set_position_reference(Cfg::StateName::Feeder, estimated_state_map[Cfg::StateName::Feeder].get_position());
            if (has_lower_feeder) governor.set_position_reference(Cfg::StateName::LowerFeeder, estimated_state_map[Cfg::StateName::LowerFeeder].get_position());

            feed = (fmod(fmod(estimated_state_map[Cfg::StateName::Feeder].get_position(), 1) + 1, 1) > 0.2)
                        ? (int)floor(estimated_state_map[Cfg::StateName::Feeder].get_position()) + 1
                        : (int)floor(estimated_state_map[Cfg::StateName::Feeder].get_position()); // reset feed to the current state
            last_feed = feed;                          // reset last feed to the current state
            // Serial.printf("Can zero\n");
        }

        // LED heartbeat -- linked to loop count to reveal slowdowns and
        // freezes.
        loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;

        // feed the watchdog to keep the loop running
        watchdog.feed();

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
    }
    return 0;
}
