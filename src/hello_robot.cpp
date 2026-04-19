#include "hello_robot.hpp"

void HelloRobot::init(){
	crash_report();
	// Execute setup functions
    pinMode(LED_BUILTIN, OUTPUT);

	Comms::comms_layer.init();

    // Configure the robot from comms data, which is filled on Hive.
    Serial.println("Configuring...");

	Comms::comms_layer.configure();
	
	const Cfg::RobotConfig& config = Comms::comms_layer.get_hive_data().config;
    
    Serial.println("Configured!");

	Serial.printf("transmitter type: %d\n", static_cast<int>(config.transmitter.transmitter_type));
	
    governor.emplace(config.states);

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
    
    estimated_state_map.emplace(config.states);
	reference_map.emplace(config.states);                   
    target_state_map.emplace(config.states);// Temp ungoverned state
    hive_state_map_offset.emplace(config.states);// Hive offset state
	
	for (const auto& state_config : config.states) {
        if (state_config.name == Cfg::StateName::LowerFeeder) {
            has_lower_feeder = true;
            break;
        }
    }

    // start the main loop watchdog
    watchdog.start();
	
}

void HelloRobot::run(){
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
}
void HelloRobot::read_telemetry(){
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
void HelloRobot::process_behaviors(){
	// manual controls on firmware
	transmitter_manager.manual_controls(*estimated_state_map, *target_state_map, not_safety_mode, feed, last_feed, has_lower_feeder);

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
void HelloRobot::update_controls(){
	// step estimates and construct estimated state
	estimator_manager.step(*estimated_state_map, override_request);
	// estimated_state_map.print();
	override_request = false;

	if ((feed - (*estimated_state_map)[Cfg::StateName::Feeder].get_position() > 2 && transmitter_manager.is_teensy_mode()) ||
		((*target_state_map)[Cfg::StateName::Feeder].get_position() - (*estimated_state_map)[Cfg::StateName::Feeder].get_position() > 2 &&
		 transmitter_manager.is_hive_mode())) {
		SystemLog.printf("Feeder is lowkey jammed. current ball count: %f, feed: %f, hive target: %f\n",
					  (*estimated_state_map)[Cfg::StateName::Feeder].get_position(), feed, (*target_state_map)[Cfg::StateName::Feeder].get_position());
		feed = (*estimated_state_map)[Cfg::StateName::Feeder].get_position() + 1;
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

	Comms::comms_layer.run();
}
void HelloRobot::check_safety(){
	bool is_slow_loop = false;

	// check whether this was a slow loop or not
	float dt = stall_timer.delta();
	if (dt > 0.002) {
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

	not_safety_mode =
		(!transmitter_manager.is_safety_mode() &&
		 Comms::comms_layer.is_configured() && !is_slow_loop && ref.ref_data.robot_performance.gimbal_power_active &&
		 !gimbal_power_recently_turned_on);
	//  SAFETY MODE
	if (not_safety_mode) {
		// SAFETY OFF
		can.write();
		//Serial.printf("Can write\n");
	} else {
		// SAFETY ON
		// TODO: Reset all controller integrators here
		can.issue_safety_mode();
if (has_lower_feeder) governor->set_position_reference(Cfg::StateName::LowerFeeder, (*estimated_state_map)[Cfg::StateName::LowerFeeder].get_position());

		float current_feed = (*estimated_state_map)[Cfg::StateName::Feeder].get_position();
		feed = (fmod(fmod(current_feed, 1) + 1, 1) > 0.2)
			? (int)floor(current_feed) + 1
			: (int)floor(current_feed); // reset feed to the current state
		last_feed = feed;                          // reset last feed to the current state
		//Serial.printf("Can zero\n");
	}
}
void HelloRobot::loop_timing(){
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
    // 1. LIVE VIEW DEFINITIONS
    // ==========================================
    enum class LiveMode { NONE, PROFILE_VIEW, TRANSMITTER, ESTIMATED_STATE, TARGET_STATE, SENSORS, HEARTBEAT };
    
    const uint8_t MAX_LIVE_VIEWS = 4;
    static LiveMode active_views[MAX_LIVE_VIEWS];
    static uint8_t num_active_views = 0;
    
    static uint32_t last_redraw_time = 0;
    static uint32_t redraw_interval = 1000; 

    // ==========================================
    // 2. LIVE VIEW RENDERER
    // ==========================================
    if (num_active_views > 0) {
        
        if (millis() - last_redraw_time >= redraw_interval) {
            Serial.print("\033[H"); // Move cursor to top-left
            
            // Loop through the array and draw the views in the exact order the user typed them
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
            Serial.println("\n[ LIVE MODE ACTIVE - PRESS ANY KEY TO EXIT ]");
            
            // Critical: \033[J clears everything *below* the cursor. 
            // This prevents "ghost text" from getting left behind if a tall view 
            // updates and suddenly becomes shorter.
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
	   add if statement to cmd elseif block below
	   If it is live add to live mode enum and switch statement above
	   as well as to cmd elseif block below
	*/
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (cli_index == 0) continue; 

            cli_buffer[cli_index] = '\0'; 
            String cmd = String(cli_buffer);
            cmd.trim();

            if (cmd == "ping") {
                Serial.println("pong! Robot is alive.");
            } 
            // --- THE PARSER ---
            else if (cmd.startsWith("live ")) {
                num_active_views = 0;
				SystemLog.is_live_view_active = true;
                redraw_interval = 1000;  // Default to slow refresh
                
                // Start reading after the word "live " (index 5)
                int startIndex = 5; 
                
                // Parse the command word by word
                while (startIndex < (int)cmd.length() && num_active_views < MAX_LIVE_VIEWS) {
                    int spaceIndex = cmd.indexOf(' ', startIndex);
                    if (spaceIndex == -1) spaceIndex = cmd.length(); // End of string
                    
                    // Extract the word
                    String arg = cmd.substring(startIndex, spaceIndex);
                    arg.trim();
                    
                    // Check the word and push the corresponding view to the stack
                    if (arg == "prof") {
                        active_views[num_active_views++] = LiveMode::PROFILE_VIEW;
                    } 
                    else if (arg == "tx") {
                        active_views[num_active_views++] = LiveMode::TRANSMITTER;
                        redraw_interval = 100; // If TX is anywhere in the stack, speed up the refresh rate
                    } 
                    else if (arg == "sensors") {
                        active_views[num_active_views++] = LiveMode::SENSORS;
                        redraw_interval = 100; // If Sensors are active, speed up the refresh rate
                    }
					else if (arg == "target_state") {
                        active_views[num_active_views++] = LiveMode::TARGET_STATE;
                        redraw_interval = 100; // If Sensors are active, speed up the refresh rate
                    }
					else if (arg == "estimated_state") {
                        active_views[num_active_views++] = LiveMode::ESTIMATED_STATE;
                        redraw_interval = 100; // If Sensors are active, speed up the refresh rate
                    }
					else if (arg == "heartbeat") {
                        active_views[num_active_views++] = LiveMode::HEARTBEAT;
                        redraw_interval = 100; // If Sensors are active, speed up the refresh rate
                    }
                    
                    // Move to the next word
                    startIndex = spaceIndex + 1;
                }

                if (num_active_views > 0) {
                    last_redraw_time = 0;    // Force immediate redraw
                    Serial.print("\033[2J"); // Clear screen to prep the canvas
                } else {
                    Serial.println("Usage: live [prof] [tx] [sensors]");
                }
            }
            else {
                Serial.println("Unknown command. Try: ping, live prof tx");
            }

            cli_index = 0; 
        } 
        else if (cli_index < 63) {
            cli_buffer[cli_index++] = c;
        }
    }
}
