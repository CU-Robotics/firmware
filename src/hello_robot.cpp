#include "hello_robot.hpp"
#ifdef PROFILER
Profiler prof; 
#endif

void HelloRobot::init() {
    crash_report();
    // Execute setup functions
    pinMode(LED_BUILTIN, OUTPUT);

    Comms::comms_layer.init();

    // Configure the robot from comms data, which is filled on Hive.
    Serial.println("Configuring...");

    Comms::comms_layer.configure();

    const Cfg::RobotConfig &config = Comms::comms_layer.get_hive_data().config;

    Serial.println("Configured!");
    Serial.printf("transmitter type: %d\n", static_cast<int>(config.transmitter.transmitter_type));
    for (const auto &state_config : config.states) {
        if (state_config.name == Cfg::StateName::LowerFeeder) {
            has_lower_feeder = true;
            break;
        }
    }

    governor.emplace(config.states);

    can.init(config.motors);

    safety::register_safety_function([&]() { can.issue_safety_mode(); });

    ref.init();
    transmitter_manager.init(config.transmitter);

    // initialize sensors
    sensor_manager.init(config);

    estimator_manager.init(config.estimators, sensor_manager, can);

    // generate controller outputs based on governed references and estimated
    // state
    controller_manager.init(config.controllers, can);

    estimated_state_map.emplace(config.states);
    reference_map.emplace(config.states);
    target_state_map.emplace(config.states);      // Temp ungoverned state
    hive_state_map_offset.emplace(config.states); // Hive offset state

    // start the main loop watchdog
    watchdog.start();
}

void HelloRobot::run() {
    Serial.println("Entering main loop...\n");
    // Main loop
    while (true) {
        // start main loop time timer
        stall_timer.start();

#ifdef PROFILER

        prof.begin("Telemetry");
        read_telemetry();
        prof.end("Telemetry");

        prof.begin("Behaviors");
        process_behaviors();
        prof.end("Behaviors");

        prof.begin("Controls");
        update_controls();
        prof.end("Controls");

        prof.begin("Safety");
        check_safety();
        prof.end("Safety");

        prof.begin("CLI");
        process_cli();
        prof.end("CLI");
		#else
		read_telemetry();
		process_behaviors();
		update_controls();
		check_safety();
		process_cli();
		#endif
		loop_timing();

		
	}
}

void HelloRobot::crash_report(){
	// over Serial in the future, we'll send this directly over comms
	if (CrashReport) {
		while (1) {
			Serial.println(CrashReport);
			Serial.println("\nReflash to clear CrashReport (and also please "
						   "fix why it crashed)");
			delay(1000);
		}
	}
}
void HelloRobot::read_telemetry() {
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
}
void HelloRobot::process_behaviors() {
    // manual controls on firmware
    transmitter_manager.manual_controls(*estimated_state_map, *target_state_map, not_safety_mode, feed, last_feed);

    // check if we want to use hive controls instead
    if (transmitter_manager.is_hive_mode()) {
        // hid_incoming.get_target_state_map(target_state_map);
        target_state_map->from_comms_packet(Comms::comms_layer.get_hive_data().target_state_data.state);
        last_feed = (*target_state_map)[Cfg::StateName::Feeder].get_position();
    }

    // override temp state if needed. Dont override in teensy mode so the sentry doesnt move during inspection
    if (Comms::comms_layer.get_hive_data().override_state_data.active && !(transmitter_manager.is_teensy_mode())) {
        // clear the request
        Comms::comms_layer.get_hive_data().override_state_data.active = false;

		SystemLog.printf("Overriding state with hive state\n");
		hive_state_map_offset->from_comms_packet(Comms::comms_layer.get_hive_data().override_state_data.state);

        *estimated_state_map = *hive_state_map_offset;
        override_request = true;
    }
}
void HelloRobot::update_controls() {
    // step estimates and construct estimated state
    estimator_manager.step(*estimated_state_map, override_request);
    // estimated_state_map.print();
    override_request = false;
    float current_feed = (*estimated_state_map)[Cfg::StateName::Feeder].get_position();
    float target_feed = (*target_state_map)[Cfg::StateName::Feeder].get_position();
    if ((feed - current_feed > 2 && transmitter_manager.is_teensy_mode()) || (target_feed - current_feed > 2 && transmitter_manager.is_hive_mode())) {
        SystemLog.printf("Feeder is lowkey jammed. current ball count: %f, feed: %f, hive target: %f\n", (*estimated_state_map)[Cfg::StateName::Feeder].get_position(), feed, (*target_state_map)[Cfg::StateName::Feeder].get_position());
        feed = current_feed + 1;
        governor->set_position_reference(Cfg::StateName::Feeder, feed);
    }

    // if first loop set target state to estimated state
    if (is_first_loop == true) {
        governor->set_reference_map(*estimated_state_map);
        is_first_loop = false;
    }

    if (transmitter_manager.mode_changed()) {
        governor->set_reference_map(*estimated_state_map);
    }
    // reference govern
    *reference_map = governor->step_reference_map(*target_state_map);

    // generate motor outputs from controls
    controller_manager.step(*reference_map, *estimated_state_map);

    target_state_map->send_to_comms<TargetState>();
    estimated_state_map->send_to_comms<EstimatedState>();

    Comms::Sendable<ConfigurationStatusData> config_status_sendable;
    config_status_sendable.data.is_configured = Comms::comms_layer.is_configured() ? 1 : 0;
    config_status_sendable.send_to_comms();

    Comms::comms_layer.run();
}
void HelloRobot::check_safety() {
    bool is_slow_loop = false;

    // check whether this was a slow loop or not
    float dt = stall_timer.delta();
    if (dt > 0.002f) {
        // zero the can bus just in case
        can.issue_safety_mode();

		SystemLog.printf("Slow loop with dt: %f, slow loop count %d\n", dt, slow_loop_counter);
		// mark this as a slow loop to trigger safety mode
		is_slow_loop = true;
		if (last_loop_slow) {
			slow_loop_counter++;
			if (slow_loop_counter > 10) {
				SystemLog.printf("Kowabunga bitches\n");
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

    not_safety_mode = (!transmitter_manager.is_safety_mode() && Comms::comms_layer.is_configured() && !is_slow_loop && ref.ref_data.robot_performance.gimbal_power_active && !gimbal_power_recently_turned_on);

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
        float current_feed = (*estimated_state_map)[Cfg::StateName::Feeder].get_position();
        governor->set_position_reference(Cfg::StateName::Feeder, current_feed);
        if (has_lower_feeder) {
            governor->set_position_reference(Cfg::StateName::LowerFeeder, (*estimated_state_map)[Cfg::StateName::LowerFeeder].get_position());
        }
        feed = (fmodf(fmodf(current_feed, 1) + 1, 1) > 0.2f) ? (int)floor(current_feed) + 1 : (int)floor(current_feed); // reset feed to the current state
        last_feed = feed;                                                                                               // reset last feed to the current state
                                                                                                                        // Serial.printf("Can zero\n");
    }
}
void HelloRobot::loop_timing() {
    // print loopc every second to verify it is still alive
    if (loopc % 1000 == 0) {
        //Serial.println(loopc);
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

void HelloRobot::process_cli() {
    // ==========================================
    // 2. LIVE VIEW RENDERER
    // ==========================================
    if (num_active_views > 0) {
        
        if (millis() - last_redraw_time >= redraw_interval) {
            Serial.print("\033[H"); // Move cursor to top-left
            
            // Loop through the array and draw the views in the order the user typed them
            for (int i = 0; i < num_active_views; i++) {
                switch (active_views[i]) {
				case LiveMode::PROFILE_VIEW:
#ifdef PROFILER
					prof.print_summary();
#endif
					break;
                        
				case LiveMode::TRANSMITTER:
					transmitter_manager.print_live_data();
					break;
                        
				case LiveMode::SENSORS:
					Serial.printf("=== LIVE SENSOR READOUT ===\n");
					sensor_manager.print_sensors_live(); 
					break;
                        
				case LiveMode::ESTIMATED_STATE:
					Serial.printf("=== LIVE ESTIMATED STATE ===\n");
					estimated_state_map->print();
					break;
				
				case LiveMode::TARGET_STATE:
					Serial.printf("=== LIVE TARGET STATE ===\n");
					target_state_map->print();
					break;

				case LiveMode::HEARTBEAT:
					Serial.printf("=== LIVE HEARTBEAT  ===\n");
					Serial.println(loopc);
					break;
                        
				default:
					break;
                }
                Serial.println(); // Add a blank line between stacked views
            }
			SystemLog.draw_dashboard_box(); // puts all non-CLI prints in neat box
            Serial.println("\n[ LIVE MODE ACTIVE - PRESS ENTER TO EXIT ]");
            
            // \033[J clears everything *below* the cursor. 
            Serial.print("\033[J"); 
            
            last_redraw_time = millis();
        }

        // Exit live mode on any keystroke
        if (Serial.available() > 0) {
            num_active_views = 0; // Empty the array
			SystemLog.is_live_view_active = false; //Turn standard scrolling prints back on
            while(Serial.available()) Serial.read(); // Flush buffer
            Serial.println("\n\n[Exited Live View]");
            cli_index = 0; 
        }
        
        return; 
    }

    // ==========================================
    // 3. NORMAL CLI PARSING
    // ==========================================
	/* Adding new commands is simple.
	   add command to command dictionary block below
	   If it is live add to live mode enum and switch statement above
	   as well as to view_dict lookup table in cmd_live function
	*/
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (cli_index == 0) continue; 

            cli_buffer[cli_index] = '\0'; 

            // --- THE COMMAND DICTIONARY ---
            static const struct {
                const char* name;
                void (HelloRobot::*execute)();
            } commands[] = {
                {"ping", &HelloRobot::cmd_ping},
                {"help", &HelloRobot::cmd_help},
                {"live", &HelloRobot::cmd_live}
            };

            // --- THE PARSER ---
            // 1. Extract the very first word
            char* cmd_str = strtok(cli_buffer, " ");
            
            if (cmd_str != nullptr) {
                bool found = false;
                
                // 2. Scan the dictionary for a match
                for (const auto& cmd : commands) {
                    if (strcmp(cmd_str, cmd.name) == 0) {
                        // 3. Execute the matched member function
                        (this->*(cmd.execute))();
                        found = true;
                        break;
                    }
                }
                
                if (!found) {
                    Serial.println("Unknown command. Try: help");
                }
            }

            cli_index = 0; 
        } 
        else if (cli_index < 63) {
            cli_buffer[cli_index++] = c;
        }
    }
}
void HelloRobot::cmd_ping() {
    Serial.println("pong! Robot is alive.");
}

void HelloRobot::cmd_help() {
                Serial.println("NAME");
                Serial.println("       Robot CLI - Control and monitor firmware");
                Serial.println();
                Serial.println("SYNOPSIS");
                Serial.println("       [command] [arguments...]");
                Serial.println();
                Serial.println("DESCRIPTION");
                Serial.println("       Provides a serial interface to interact with the robot, check");
                Serial.println("       connection status, and launch live, real-time data dashboards.");
                Serial.println();
                Serial.println("COMMANDS");
                Serial.println("       ping");
                Serial.println("              Replies with 'pong!' to verify the serial connection is active.");
                Serial.println();
                Serial.println("       live [view1] [view2] ...");
                Serial.println("              Launches a live updating dashboard with the specified views.");
                Serial.println("              Views are stacked vertically in the order provided.");
                Serial.println("              Press ENTER to exit live mode.");
                Serial.println();
                Serial.println("              Available views:");
                Serial.println("                prof            : Execution time profiler (only available if running make debug) ");
                Serial.println("                tx              : Real-time radio transmitter inputs");
                Serial.println("                sensors         : Real-time readouts from all configured sensors");
                Serial.println("                estimated_state : The robot's current estimated state map");
                Serial.println("                target_state    : The robot's current target state map");
                Serial.println("                heartbeat       : The main loop counter (loopc)");
                Serial.println();
                Serial.println("       help");
                Serial.println("              Displays this manual.");
}
void HelloRobot::cmd_live() {
    num_active_views = 0;
    SystemLog.is_live_view_active = true;
    redraw_interval = 1000;
    
    struct LiveViewMap {
        const char* name;
        LiveMode mode;
        uint32_t interval;
    };
    
    static const LiveViewMap view_dict[] = {
        {"prof",            LiveMode::PROFILE_VIEW,    1000}, 
        {"tx",              LiveMode::TRANSMITTER,     100},
        {"sensors",         LiveMode::SENSORS,         100},
        {"target_state",    LiveMode::TARGET_STATE,    100},
        {"estimated_state", LiveMode::ESTIMATED_STATE, 100},
        {"heartbeat",       LiveMode::HEARTBEAT,       100}
    };

	// --- THE PARSER ---
    char* token;
    while ((token = strtok(NULL, " ")) != NULL && num_active_views < MAX_LIVE_VIEWS) {
        
        // Scan the dictionary for a matching view
        for (const auto& view : view_dict) {
            if (strcmp(token, view.name) == 0) {
                // Add the view to the active stack
                active_views[num_active_views++] = view.mode;
                
                // If this view requires a faster refresh rate, upgrade the global interval
                if (view.interval < redraw_interval) {
                    redraw_interval = view.interval;
                }
                break; // Found a match, break the inner loop to grab the next word
            }
        }
    }

    if (num_active_views > 0) {
        last_redraw_time = 0;    
        Serial.print("\033[2J"); 
    } else {
        Serial.println("Usage: live [prof] [tx] [sensors] [estimated_state] [target_state] [heartbeat]");
    }
}
