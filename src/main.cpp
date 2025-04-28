#include <Arduino.h>

#include "utils/logger.hpp"

#include "comms/comms_layer.hpp"
#include "git_info.h"

#include "utils/profiler.hpp"
#include "sensors/StereoCamTrigger.hpp"
#include "controls/estimator_manager.hpp"
#include "controls/controller_manager.hpp"

#include <TeensyDebug.h>
#include "sensors/LEDBoard.hpp"
#include "SensorManager.hpp"

#include "utils/watchdog.hpp"
#include "comms/data/sendable.hpp"
#include "comms/data/hive_data.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Declare global objects
DR16 dr16;
CANManager can;
RefSystem* ref;
ACS712 current_sensor;
Comms::CommsLayer comms_layer;

StereoCamTrigger stereoCamTrigger(60);

ConfigLayer config_layer;

Profiler prof;

SensorManager sensor_manager;
EstimatorManager estimator_manager;
ControllerManager controller_manager;

Governor governor;

Watchdog watchdog;

// DONT put anything else in this function. It is not a setup function
void print_logo() {
    if (Serial) {
        logger.println(LogDestination::Serial, "TEENSY SERIAL START\n\n");
        logger.print(LogDestination::Serial, "\033[1;33m");
        logger.println(LogDestination::Serial, "                  .:^!?!^.                        ");
        logger.println(LogDestination::Serial, "           .:~!?JYYYJ?7?Y5Y7!!.                   ");
        logger.println(LogDestination::Serial, "         :?5YJ?!~:.      ^777YP?.                 ");
        logger.println(LogDestination::Serial, "         5G~                  ~YP?:               ");
        logger.println(LogDestination::Serial, "         7P5555Y:               ^YP?:....         ");
        logger.println(LogDestination::Serial, "        ~55J7~^.   ..    .        ^JYYYYYYYYYJJ!. ");
        logger.println(LogDestination::Serial, "        YG^     !Y5555J:^PJ    Y5:      ...::^5G^ ");
        logger.println(LogDestination::Serial, "       :GY    .YG?^..^~ ~GY    5G^ ^!~~^^^!!~7G?  ");
        logger.println(LogDestination::Serial, " .!JYYY5G!    7BJ       ~GY    5G^ ~??JJJY555GP!  ");
        logger.println(LogDestination::Serial, "^55!^:.^~.    ^PP~   .: ^GP:  ^PP:           :7PY.");
        logger.println(LogDestination::Serial, "YG^            :JP5YY55: ~YP55PY^              ~GJ");
        logger.println(LogDestination::Serial, "?G~      .?7~:   .^~~^.    .^:.                :G5");
        logger.println(LogDestination::Serial, ".5P^     7BYJ5YJ7^.                          .~5P^");
        logger.println(LogDestination::Serial, " .JPJ!~!JP?  .:~?PP^            .:.    .^!JYY5Y!. ");
        logger.println(LogDestination::Serial, "   :!???!:       5P.         .!Y5YYYJ?Y5Y?!^:.    ");
        logger.println(LogDestination::Serial, "                 7G7        7GY!. .:~!^.          ");
        logger.println(LogDestination::Serial, "                  JG!      :G5                    ");
        logger.println(LogDestination::Serial, "                   7PY!^^~?PY:                    ");
        logger.println(LogDestination::Serial, "                    .!JJJJ?^                      ");
        logger.print(LogDestination::Serial, "\033[0m");
        logger.println(LogDestination::Serial, "\n\033[1;92mFW Ver. 2.1.0");
        logger.printf(LogDestination::Serial, "\nLast Built: %s at %s", __DATE__, __TIME__);
        logger.printf(LogDestination::Serial, "\nGit Hash: %s", GIT_COMMIT_HASH);
        logger.printf(LogDestination::Serial, "\nGit Branch: %s", GIT_BRANCH);
        logger.printf(LogDestination::Serial, "\nCommit Message: %s", GIT_COMMIT_MSG);
        logger.printf(LogDestination::Serial, "\nRandom Num: %x", ARM_DWT_CYCCNT);
        logger.println(LogDestination::Serial, "\033[0m\n");
    }
}

// Master loop
int main() {
    uint32_t loopc = 0; // Loop counter for heartbeat

    Serial.begin(115200); // the serial monitor is actually always active (for debug use logger.println & tycmd)
    debug.begin(SerialUSB1);
    
    print_logo();

    // check to see if there is a crash report, and if so, print it repeatedly over Serial
    // in the future, we'll send this directly over comms
    if (CrashReport) {
        while (1) {
            logger.println(CrashReport);
            logger.println("\nReflash to clear CrashReport (and also please fix why it crashed)");
            delay(1000);
        }
    }

    // Execute setup functions
    pinMode(LED_BUILTIN, OUTPUT);
    
    //initialize objects
    can.init();
    dr16.init();
    comms_layer.init();
    ref = sensor_manager.get_ref();

    // Config config
    logger.println("Configuring...");
    const Config* config = config_layer.configure(&comms_layer);
    logger.println("Configured!");

    // configure motors
    can.configure(config->motor_info);

    // initialize sensors
    sensor_manager.init(config);

    //estimate micro and macro state
    estimator_manager.init(&can, config, &sensor_manager);

    //generate controller outputs based on governed references and estimated state
    controller_manager.init(&can, config);

    //set reference limits in the reference governor
    governor.set_reference_limits(config->set_reference_limits);

    // print all of config
    config->print();

    // variables for use in main
    float temp_state[STATE_LEN][3] = { 0 }; // Temp state array
    float temp_micro_state[CAN_MAX_MOTORS][MICRO_STATE_LEN] = { 0 }; // Temp micro state array
    float temp_reference[STATE_LEN][3] = { 0 }; //Temp governed state
    float target_state[STATE_LEN][3] = { 0 }; //Temp ungoverned state
    float hive_state_offset[STATE_LEN][3] = { 0 }; //Hive offset state
    bool override_request = false;
    // float motor_inputs[CAN_MAX_MOTORS] = { 0 }; //Array for storing controller outputs to send to CAN

    // manual controls variables
    float vtm_pos_x = 0;
    float vtm_pos_y = 0;
    float dr16_pos_x = 0;
    float dr16_pos_y = 0;
    float pos_offset_x = 0;
    float pos_offset_y = 0;

    // param to specify whether this is the first loop
    int count_one = 0;

    // whether we are in hive mode or not
    bool hive_toggle = false;

    // main loop timers
    Timer loop_timer;
    Timer stall_timer;
    Timer control_input_timer;
    
    // start the main loop watchdog
    watchdog.start();

    logger.println("Entering main loop...\n");

    uint8_t log_buffer_copy[LOGGER_BUFFER_SIZE];

    // Main loop
    while (true) {
        // start main loop time timer
        stall_timer.start();
        
        // read sensors
        sensor_manager.read();
        // read CAN and DR16 -- These are kept out of sensor manager for safety reasons
        can.read();
        dr16.read();

        sensor_manager.send_sensor_data_to_comms();

        // check whether this packet is a config packet
        if (comms_layer.get_hive_data().config_section.request_bit == 1) {
            logger.println("\n\nConfig request received, reconfiguring from comms!\n\n");
            // trigger safety mode
            can.issue_safety_mode();
            config_layer.reconfigure(&comms_layer);
        }

        // print loopc every second to verify it is still alive
        if (loopc % 1000 == 0) {
            Serial.println(loopc);
        }

        // manual controls on firmware
        float delta = control_input_timer.delta();
        dr16_pos_x += dr16.get_mouse_x() * 0.05 * delta;
        dr16_pos_y += dr16.get_mouse_y() * 0.05 * delta;

        vtm_pos_x += ref->ref_data.kbm_interaction.mouse_speed_x * 0.05 * delta;
        vtm_pos_y += ref->ref_data.kbm_interaction.mouse_speed_y * 0.05 * delta;

        float chassis_vel_x = 0;
        float chassis_vel_y = 0;
        float chassis_pos_x = 0;
        float chassis_pos_y = 0;
        if (config->governor_types[0] == 2) {   // if we should be controlling velocity
            chassis_vel_x = dr16.get_l_stick_y() * 5.4
                + (-ref->ref_data.kbm_interaction.key_w + ref->ref_data.kbm_interaction.key_s) * 2.5
                + (-dr16.keys.w + dr16.keys.s) * 2.5;
            chassis_vel_y = -dr16.get_l_stick_x() * 5.4
                + (ref->ref_data.kbm_interaction.key_d - ref->ref_data.kbm_interaction.key_a) * 2.5
                + (dr16.keys.d - dr16.keys.a) * 2.5;
        } else if (config->governor_types[0] == 1) { // if we should be controlling position
            chassis_pos_x = dr16.get_l_stick_x() * 2 + pos_offset_x;
            chassis_pos_y = dr16.get_l_stick_y() * 2 + pos_offset_y;
        }

        float chassis_spin = dr16.get_wheel() * 25;
        float pitch_target = 1.57
            + -dr16.get_r_stick_y() * 0.3
            + dr16_pos_y
            + vtm_pos_y;
        float yaw_target = -dr16.get_r_stick_x() * 1.5
            - dr16_pos_x
            - vtm_pos_x;
        float fly_wheel_target = (dr16.get_r_switch() == 1 || dr16.get_r_switch() == 3) ? 18 : 0; //m/s
        float feeder_target = (((dr16.get_l_mouse_button() || ref->ref_data.kbm_interaction.button_left) && dr16.get_r_switch() != 2) || dr16.get_r_switch() == 1) ? 10 : 0;

        // set manual controls
        target_state[0][0] = chassis_pos_x;
        target_state[0][1] = chassis_vel_x;
        target_state[1][0] = chassis_pos_y;
        target_state[1][1] = chassis_vel_y;
        target_state[2][1] = chassis_spin;
        target_state[3][0] = yaw_target;
        target_state[3][1] = 0;
        target_state[4][0] = pitch_target;
        target_state[4][1] = 0;

        target_state[5][1] = fly_wheel_target;
        target_state[6][1] = feeder_target;
        target_state[7][0] = 1;

        // if the left switch is all the way down use Hive controls
        if (dr16.get_l_switch() == 2) {
            // hid_incoming.get_target_state(target_state);
            memcpy(target_state, comms_layer.get_hive_data().target_state.state, sizeof(target_state));

            // if you just switched to hive controls, set the reference to the current state
            if (hive_toggle) {
                governor.set_reference(temp_state);
                hive_toggle = false;
            }
        }

        // when in teensy control mode reset hive toggle
        if (dr16.get_l_switch() == 3) {
            if (!hive_toggle) {
                pos_offset_x = temp_state[0][0];
                pos_offset_y = temp_state[1][0];
            }
            hive_toggle = true;
        }

        // print dr16
        // Serial.printf("DR16:\n\t");
        // dr16.print();

        // Serial.printf("Target state:\n");
        // for (int i = 0; i < 8; i++) {
        //     Serial.printf("\t%d: %f %f %f\n", i, target_state[i][0], target_state[i][1], target_state[i][2]);
        // }
        
        // override temp state if needed
        if (comms_layer.get_hive_data().override_state.active) {
            // clear the request
            comms_layer.get_hive_data().override_state.active = false;
            
            logger.printf("Overriding state with hive state\n");
            memcpy(hive_state_offset, comms_layer.get_hive_data().override_state.state, sizeof(hive_state_offset));
            memcpy(temp_state, hive_state_offset, sizeof(hive_state_offset));
            override_request = true;
        }

        // step estimates and construct estimated state
        estimator_manager.step(temp_state, temp_micro_state, override_request);
        override_request = false;

        // if first loop set target state to estimated state
        if (count_one == 0) {
            temp_state[7][0] = 0;
            governor.set_reference(temp_state);
            count_one++;
        }

        // Serial.printf("Estimated state:\n");
        // for (int i = 0; i < 8; i++) {
        //     Serial.printf("\t%d: %f %f %f\n", i, temp_state[i][0], temp_state[i][1], temp_state[i][2]);
        // }

        // give the sensors the current estimated state
        sensor_manager.set_estimated_state(temp_state);

        // reference govern
        governor.set_estimate(temp_state);
        governor.step_reference(target_state, config->governor_types);
        governor.get_reference(temp_reference);

        // Serial.printf("Reference state:\n");
        // for (int i = 0; i < 8; i++) {
        //     Serial.printf("\t%d: %f %f %f\n", i, temp_reference[i][0], temp_reference[i][1], temp_reference[i][2]);
        // }

        // generate motor outputs from controls
        controller_manager.step(temp_reference, temp_state, temp_micro_state);

        // can.print_state();

        // construct ref data packet
        CommsRefData ref_data = ref->get_data_for_comms();
        Comms::Sendable<CommsRefData> ref_data_sendable = ref_data;
        ref_data_sendable.send_to_comms();

        Comms::Sendable<EstimatedState> estimated_state;
        memcpy(estimated_state.data.state, temp_state, sizeof(temp_state));
        estimated_state.data.time = millis() / 1000.0;
        estimated_state.send_to_comms();

        Comms::Sendable<DR16Data> dr16_sendable;
        dr16_sendable.data.l_mouse_button = dr16.get_l_mouse_button();
        dr16_sendable.data.r_mouse_button = dr16.get_r_mouse_button();
        dr16_sendable.data.l_switch = dr16.get_l_switch();
        dr16_sendable.data.r_switch = dr16.get_r_switch();
        dr16_sendable.data.l_stick_x = dr16.get_l_stick_x();
        dr16_sendable.data.l_stick_y = dr16.get_l_stick_y();
        dr16_sendable.data.r_stick_x = dr16.get_r_stick_x();
        dr16_sendable.data.r_stick_y = dr16.get_r_stick_y();
        dr16_sendable.data.wheel = dr16.get_wheel();
        dr16_sendable.data.mouse_x = dr16.get_mouse_x();
        dr16_sendable.data.mouse_y = dr16.get_mouse_y();
        dr16_sendable.data.keys.raw = *(uint16_t*)(dr16.get_raw() + 14);
        dr16_sendable.send_to_comms();

        comms_layer.run();

        bool is_slow_loop = false;

        // check whether this was a slow loop or not
        float dt = stall_timer.delta();
        logger.printf("Loop %d, dt: %f\n", loopc, dt);
        if (dt > 0.002) {
            // zero the can bus just in case
            can.issue_safety_mode();

            logger.printf("Slow loop with dt: %f\n", dt);
            // mark this as a slow loop to trigger safety mode
            is_slow_loop = true;
        }

        //  SAFETY MODE
        if (dr16.is_connected() && (dr16.get_l_switch() == 2 || dr16.get_l_switch() == 3) && config_layer.is_configured() && !is_slow_loop) {
            // SAFETY OFF
            can.write();
            // logger.printf("Can write\n");
        } else {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.issue_safety_mode();
            // logger.printf("Can zero\n");
        }

        // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;

        // feed the watchdog to keep the loop running
        watchdog.feed();

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));

        // Print logger's buffer at the end of every loop
        uint32_t bytes_copied = logger.grab_log_data(LOGGER_BUFFER_SIZE, log_buffer_copy);
        if (bytes_copied > 0) {
            Serial.write(log_buffer_copy, bytes_copied);
        }
    }

    return 0;
}
