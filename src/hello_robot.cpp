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
	
    // start the main loop watchdog
    watchdog.start();
	
}

void HelloRobot::run(){
    Serial.println("Entering main loop...\n");
	// Main loop
    while (true) {
        // start main loop time timer
        stall_timer.start();
		
		read_telemetry();
		process_behaviors();
		update_controls();
		check_safety();
		process_cli();
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

		Serial.printf("Overriding state with hive state\n");
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
		Serial.printf("Feeder is lowkey jammed. current ball count: %f, feed: %f, hive target: %f\n",
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
		governor->set_position_reference(Cfg::StateName::Feeder, (*estimated_state_map)[Cfg::StateName::Feeder].get_position());
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

void HelloRobot::process_cli(){
	// Read all available characters in the hardware buffer
    while (Serial.available() > 0) {
        char c = Serial.read();
        // If the character is a newline or carriage return, the command is complete
        if (c == '\n' || c == '\r') {
            if (cli_index == 0) continue; // Ignore empty lines

            cli_buffer[cli_index] = '\0'; // Null-terminate the string
            String cmd = String(cli_buffer);
			cmd.trim();
            // ==========================================
            // COMMAND DICTIONARY
            // ==========================================
            if (cmd == "ping") {
                Serial.println("pong! Robot is alive.");
            } 
			else if (cmd == "print tx") {
                // Access the transmitter data
                Serial.printf(" \rSafety: %d | L_X: %.2f | L_Y: %.2f | R_X: %.2f | R_Y: %.2f", 
                              transmitter_manager.transmitter.is_safety_mode(),
                              transmitter_manager.get_l_stick_x(),
                              transmitter_manager.get_l_stick_y(),
                              transmitter_manager.get_r_stick_x(),
                              transmitter_manager.get_r_stick_y());
            }
			else if (cmd == "print dt") {
                Serial.printf("Last loop dt: %f seconds\n", stall_timer.delta());
			} 
            else {
                Serial.println("Unknown command. Try: ping");
			}

            // Reset the buffer for the next command
            cli_index = 0; 
        } 
        else if (cli_index < 63) {
            // Add the typed character to the buffer
            cli_buffer[cli_index++] = c;
        }
    }
}
