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

#include "sensors/ET16S.hpp"
#include "sensors/Transmitter.hpp"
#include "sensors/d200.hpp"

#include "controls/controller_manager.hpp"
#include "controls/estimator_manager.hpp"
#include "sensors/StereoCamTrigger.hpp"
#include "sensors/dr16.hpp"
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
Transmitter *transmitter = nullptr;

Comms::CommsLayer comms_layer;

Profiler prof;

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
    // Determine which transmitter is in use and instantiate its respective object.
    // This allows for 'transmitter' to be used everywhere dr16 would be used
    TransmitterType transmitter_type = transmitter->who_am_i();
    if (transmitter_type == TransmitterType::DR16) {
        transmitter = new DR16;
    } else if (transmitter_type == TransmitterType::ET16S) {
        transmitter = new ET16S;
    }

    comms_layer.init();

    comms_layer.configure();
    
    // Config config
    Serial.println("Configuring...");
    const Cfg::RobotConfig& config = comms_layer.get_hive_data().config;
    
    Serial.println("Configured!");
    
    Governor governor(config.states);

    // initialize objects
    can.init(config.motors);
    
    safety::register_safety_function([&](){ can.issue_safety_mode(); });

    ref.init();
    transmitter->init();
    
    // initialize sensors
    sensor_manager.init(config);
    
    // estimate micro and rmacro state
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

    // manual controls variables
    float vtm_pos_x = 0;
    float vtm_pos_y = 0;
    float transmitter_pos_x = 0;
    float transmitter_pos_y = 0;
    float pos_offset_x = 0;
    float pos_offset_y = 0;
    float feed = 0;
    float last_feed = 0;

    // param to specify whether this is the first loop
    int count_one = 0;
    
    // whether we are in hive mode or not
    bool hive_toggle = false;
    bool safety_toggle = false;
    bool not_safety_mode = false;
    bool last_gimbal_power = false; // used to detect gimbal power changes
    bool last_loop_slow = false;    // used to detect multiple slow loops in a row
    int slow_loop_counter = 0;      // used to count slow loops in a row

    // get the pitch min and max, and shift them to be centered around 0
    float pitch_min = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.min;
    float pitch_max = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.max;
    float pitch_average = 0.5 * (pitch_min + pitch_max);
    pitch_min -= pitch_average;
    pitch_max -= pitch_average;
    // int last_switch = 0;

    // main loop timers
    Timer loop_timer;
    Timer timer;
    Timer stall_timer;
    Timer control_input_timer;
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
        transmitter->read();
        transmitter->send_to_comms();

        // read sensors and send to comms
        // this happens in one function call 
        sensor_manager.read();
        sensor_manager.send_to_comms();
        
        // check whether this packet is a config packet
        // if (comms_layer.get_hive_data().config_section.request_bit == 1) {
        //     Serial.println("\n\nConfig request received, reconfiguring from comms!\n\n");
        //     // trigger safety mode
        //     can.issue_safety_mode();
        //     config_layer.reconfigure(&comms_layer);
        // }

        // print loopc every second to verify it is still alive
        if (loopc % 1000 == 0) {
            Serial.println(loopc);
        }

        // manual controls on firmware
            std::optional<Transmitter::Keys> transmitter_keys = transmitter->get_keys();
            std::optional<int> mouse_x = transmitter->get_mouse_x();
            std::optional<int> mouse_y = transmitter->get_mouse_y();
            std::optional<bool> l_mouse_button = transmitter->get_l_mouse_button();
            // std::optional<bool> r_mouse_button = transmitter->get_r_mouse_button();

            float delta = control_input_timer.delta();
            if (mouse_x.has_value() && mouse_y.has_value()) {
                transmitter_pos_x += mouse_x.value() * 0.05 * delta;
                transmitter_pos_y += mouse_y.value() * 0.05 * delta;
            }

            vtm_pos_x += ref.ref_data.kbm_interaction.mouse_speed_x * 0.05 * delta;
            vtm_pos_y += ref.ref_data.kbm_interaction.mouse_speed_y * 0.05 * delta;

            // clamp to pitch limits
            if (transmitter_pos_y < pitch_min) {
                transmitter_pos_y = pitch_min;
            }
            if (transmitter_pos_y > pitch_max) {
                transmitter_pos_y = pitch_max;
            }

            float chassis_vel_x = 0;
            float chassis_vel_y = 0;
            float chassis_pos_x = 0;
            float chassis_pos_y = 0;

            if (estimated_state_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Velocity) { // if we should be controlling velocity

                chassis_vel_x = transmitter->get_l_stick_y() * 5.4 +
                                (-ref.ref_data.kbm_interaction.key_w + ref.ref_data.kbm_interaction.key_s) * 2.5;

                if (transmitter_keys.has_value()) {
                    chassis_vel_x += (-transmitter_keys.value().w + transmitter_keys.value().s) * 2.5;
                }

                chassis_vel_y = -transmitter->get_l_stick_x() * 5.4 +
                                (ref.ref_data.kbm_interaction.key_d - ref.ref_data.kbm_interaction.key_a) * 2.5;

                if (transmitter_keys.has_value()) {
                    chassis_vel_y += (transmitter_keys.value().d - transmitter_keys.value().a) * 2.5;
                }
            } else if (estimated_state_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Position) { // if we should be controlling position
                chassis_pos_x = transmitter->get_l_stick_x() * 2 + pos_offset_x;
                chassis_pos_y = transmitter->get_l_stick_y() * 2 + pos_offset_y;
            }

            float chassis_spin = transmitter->get_wheel() * 25;
            float pitch_target = 1.57 + -transmitter->get_r_stick_y() * 0.3 + transmitter_pos_y + vtm_pos_y;
            float yaw_target = -transmitter->get_r_stick_x() * 1.5 - transmitter_pos_x - vtm_pos_x;

            float fly_wheel_target =
                (transmitter->get_r_switch() == SwitchPos::FORWARD || transmitter->get_r_switch() == SwitchPos::MIDDLE) ? 18 : 0; // m/s
            // if the right switch is forward, and either the left mouse button is pressed or the right switch is not
            // backward, set the feeder to something. Otherwise, set it to 0
            float feeder_target = (((l_mouse_button.has_value() || ref.ref_data.kbm_interaction.button_left) &&
                                    transmitter->get_r_switch() != SwitchPos::BACKWARD) || transmitter->get_r_switch() == SwitchPos::FORWARD) ? 10 : 0;
            if (estimated_state_map[Cfg::StateName::Feeder].config().governor_type == Cfg::StateOrder::Position) {
                float dt2 = timer.delta();
                if (dt2 > 0.1)
                    dt2 = 0;
                // check if the shooter is active
                if (not_safety_mode && ref.ref_data.robot_performance.shooter_power_active)
                    feed += feeder_target * dt2;
                target_state_map[Cfg::StateName::Feeder].set_position((int)feed);
            } else { 
                target_state_map[Cfg::StateName::Feeder].set_velocity(feeder_target);
            }
            // if (transmitter->get_r_switch() == 1 && last_switch != 1) {
            //     feed++;
            // }
            // last_switch = transmitter->get_r_switch();
            // set manual controls
            target_state_map[Cfg::StateName::ChassisX].set_position(chassis_pos_x);
            target_state_map[Cfg::StateName::ChassisX].set_velocity(chassis_vel_x);
            target_state_map[Cfg::StateName::ChassisY].set_position(chassis_pos_y);
            target_state_map[Cfg::StateName::ChassisY].set_velocity(chassis_vel_y);
            target_state_map[Cfg::StateName::ChassisHeading].set_velocity(chassis_spin);
            target_state_map[Cfg::StateName::GimbalYaw].set_position(yaw_target);
            target_state_map[Cfg::StateName::GimbalYaw].set_velocity(0);
            target_state_map[Cfg::StateName::GimbalPitch].set_position(pitch_target);
            target_state_map[Cfg::StateName::GimbalPitch].set_velocity(0);
            target_state_map[Cfg::StateName::Flywheels].set_velocity(fly_wheel_target);

            // if the left switch is all the way down use Hive controls

            if (transmitter->get_l_switch() == SwitchPos::BACKWARD) {
                // hid_incoming.get_target_state_map(target_state_map);
                target_state_map.from_comms_packet(comms_layer.get_hive_data().target_state_data.state);
                last_feed = target_state_map[Cfg::StateName::Feeder].get_position();
                // if you just switched to hive controls, set the reference to the
                // current state'
                if (hive_toggle) {
                    governor.set_reference_map(estimated_state_map);
                    hive_toggle = false;
                }
            }

            // when in teensy control mode reset hive toggle
            if (transmitter->get_l_switch() == SwitchPos::MIDDLE) {
                if (!hive_toggle || !safety_toggle) {
                    pos_offset_x = estimated_state_map[Cfg::StateName::ChassisX].get_position();
                    pos_offset_y = estimated_state_map[Cfg::StateName::ChassisY].get_position();
                    feed = last_feed;
                    governor.set_reference_map(estimated_state_map);
                }
                hive_toggle = true;
                safety_toggle = true;
            }

        // print transmitter

        // Serial.printf("transmitter:\n\t");
        // transmitter->print();

        // Serial.printf("Target state:\n");
        // for (int i = 0; i < 8; i++) {
        //     Serial.printf("\t%d: %f %f %f\n", i, target_state_map[i][0],
        //     target_state_map[i][1], target_state_map[i][2]);
        // }

        // override temp state if needed. Dont override in teensy mode so the sentry doesnt move during inspection
        if (comms_layer.get_hive_data().override_state_data.active && !(transmitter->get_l_switch() == SwitchPos::MIDDLE)) {
            // clear the request
            comms_layer.get_hive_data().override_state_data.active = false;

            Serial.printf("Overriding state with hive state\n");
            hive_state_map_offset.from_comms_packet(comms_layer.get_hive_data().override_state_data.state);

            estimated_state_map = hive_state_map_offset;
            override_request = true;
        }

        // step estimates and construct estimated state
        estimator_manager.step(estimated_state_map, override_request);
        override_request = false;

        if ((feed - estimated_state_map[Cfg::StateName::Feeder].get_position() > 2 && transmitter->get_l_switch() == SwitchPos::MIDDLE) ||
            (target_state_map[Cfg::StateName::Feeder].get_position() - estimated_state_map[Cfg::StateName::Feeder].get_position() > 2 &&
             transmitter->get_l_switch() == SwitchPos::BACKWARD)) {
            Serial.printf("Feeder is lowkey jammed. current ball count: %f, feed: %f, hive target: %f\n",
                            estimated_state_map[Cfg::StateName::Feeder].get_position(), feed, target_state_map[Cfg::StateName::Feeder].get_position());
            feed = estimated_state_map[Cfg::StateName::Feeder].get_position() + 1;
            governor.set_position_reference(Cfg::StateName::Feeder, feed);
        }

        // if first loop set target state to estimated state
        if (count_one == 0) {
            governor.set_reference_map(estimated_state_map);
            // print temp state
            // for (int i = 0; i < 8; i++) {
            //     Serial.printf("\t%d: %f %f %f\n", i, estimated_state_map[i][0], estimated_state_map[i][1], estimated_state_map[i][2]);
            // }
            count_one++;
        }

        // reference govern
        reference_map = governor.step_reference_map(target_state_map);

        // generate motor outputs from controls
        controller_manager.step(reference_map, estimated_state_map);

        // can.print_state();

        target_state_map.send_to_comms<TargetState>();
        estimated_state_map.send_to_comms<EstimatedState>();

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

        if (!last_gimbal_power && ref.ref_data.robot_performance.gimbol_power_active) {
            gimbal_power_timer.start();
        }
        last_gimbal_power = ref.ref_data.robot_performance.gimbol_power_active;
        bool gimbal_power_recently_turned_on = gimbal_power_timer.get_elapsed_micros_no_restart() < 3000000;

        not_safety_mode =
            (transmitter->is_connected() &&
             (transmitter->get_l_switch() == SwitchPos::BACKWARD || transmitter->get_l_switch() == SwitchPos::MIDDLE) &&
             comms_layer.is_configured() && !is_slow_loop && ref.ref_data.robot_performance.gimbol_power_active &&
             !gimbal_power_recently_turned_on);
        //  SAFETY MODE
        if (not_safety_mode) {
            // SAFETY OFF
            can.write();
            // Serial.printf("Can write\n");
            // Serial.printf("Can write\n");
        } else {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.issue_safety_mode();
            governor.set_position_reference(Cfg::StateName::Feeder, estimated_state_map[Cfg::StateName::Feeder].get_position());

            feed = (fmod(fmod(estimated_state_map[Cfg::StateName::Feeder].get_position(), 1) + 1, 1) > 0.2)
                        ? (int)floor(estimated_state_map[Cfg::StateName::Feeder].get_position()) + 1
                        : (int)floor(estimated_state_map[Cfg::StateName::Feeder].get_position()); // reset feed to the current state
            last_feed = feed;                          // reset last feed to the current state
            // Serial.printf("Can zero\n");
            safety_toggle = false; // reset hive toggle
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
