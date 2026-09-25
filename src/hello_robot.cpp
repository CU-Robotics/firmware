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

    safety::register_safety_function([&]() { can.zero_all_motors(); });

    ref.init();
    transmitter_manager.init(config.transmitter);

    // initialize sensors
	sensor_manager.init(config, &estimated_state_array_interrupt_safe);
	// Begin cycle of reading sensor data
	sensor_manager.request_read();

    estimator_manager.init(config.estimators, sensor_manager, can);

    // generate controller outputs based on governed references and estimated
    // state
    controller_manager.init(config.controllers, can, config.states);

    estimated_state_array.emplace(config.states);
    estimated_state_array_interrupt_safe = std::make_unique<RobotStateArray>(config.states);
    reference_array.emplace(config.states);
    target_state_array.emplace(config.states);      // Temp ungoverned state
    hive_state_array_offset.emplace(config.states); // Hive offset state
    
    cli.init(transmitter_manager, sensor_manager, *estimated_state_array, *target_state_array, loopc);
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
        prof.begin("Comms");
        update_comms();
		prof.end("Comms"); 
        prof.begin("Safety");
        check_safety();
        prof.end("Safety");

        prof.begin("CLI");
		cli.process();
        prof.end("CLI");
#else
		read_telemetry();
		process_behaviors();
		update_controls();
		update_comms();
		check_safety();
		cli.process();
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
void HelloRobot::read_telemetry(){
	// read sensors and send to comms
	// this happens in one function call 
	sensor_manager.read();
	sensor_manager.send_to_comms();
	
	// read CAN and send motor states to comms
	can.read();
	can.send_to_comms();

    // read ref and send to comms
    ref.read();
    ref.send_to_comms();

	// read transmitter and send to comms
	transmitter_manager.read();
	transmitter_manager.send_to_comms();
	
	// Begin Sensor DMA transfer for next loop
	sensor_manager.request_read();
		
}
void HelloRobot::process_behaviors() {
    // manual controls on firmware
    transmitter_manager.manual_controls(*estimated_state_array, *target_state_array, motors_armed, feed, last_feed);

    // check if we want to use hive controls instead
    if (transmitter_manager.is_hive_mode()) {
        // hid_incoming.get_target_state_array(target_state_array);
        target_state_array->from_comms_packet(Comms::comms_layer.get_hive_data().target_state_data.state);
        last_feed = (*target_state_array)[Cfg::StateName::Feeder].get_position();
    }

    // override temp state if needed. Dont override in teensy mode so the sentry doesnt move during inspection
    if (Comms::comms_layer.get_hive_data().override_state_data.active && !(transmitter_manager.is_teensy_mode())) {
        // clear the request
        Comms::comms_layer.get_hive_data().override_state_data.active = false;

		SystemLog.info(Subsystem::GENERAL,"Overriding state with hive state\n");
		hive_state_array_offset->from_comms_packet(Comms::comms_layer.get_hive_data().override_state_data.state);

        *estimated_state_array = *hive_state_array_offset;
        override_request = true;
    }
}
void HelloRobot::update_controls() {
    // step estimates and construct estimated state
    estimator_manager.step(*estimated_state_array, override_request);
    // estimated_state_array.print();

    noInterrupts();
    *estimated_state_array_interrupt_safe = *estimated_state_array;
    interrupts();
    
    override_request = false;
    float current_feed = (*estimated_state_array)[Cfg::StateName::Feeder].get_position();
    float target_feed = (*target_state_array)[Cfg::StateName::Feeder].get_position();
    if ((feed - current_feed > 2 && transmitter_manager.is_teensy_mode()) || (target_feed - current_feed > 2 && transmitter_manager.is_hive_mode())) {
        SystemLog.error(Subsystem::GENERAL,"Feeder is lowkey jammed. current ball count: %f, feed: %f, hive target: %f\n", (*estimated_state_array)[Cfg::StateName::Feeder].get_position(), feed, (*target_state_array)[Cfg::StateName::Feeder].get_position());
        feed = current_feed + 1;
        governor->set_position_reference(Cfg::StateName::Feeder, feed);
    }

    // if first loop set target state to estimated state
    if (is_first_loop == true) {
        governor->set_reference_array(*estimated_state_array);
        is_first_loop = false;
    }

    if (transmitter_manager.mode_changed()) {
        governor->set_reference_array(*estimated_state_array);
    }
    // reference govern
    *reference_array = governor->step_reference_array(*target_state_array);

    // generate motor outputs from controls
    controller_manager.step(*reference_array, *estimated_state_array, *target_state_array);
}
void HelloRobot::update_comms() {
    target_state_array->send_to_comms<TargetState>();
    reference_array->send_to_comms<ReferenceState>();
    estimated_state_array->send_to_comms<EstimatedState>();
    Comms::Sendable<ConfigurationStatusData> config_status_sendable;
    config_status_sendable.data.is_configured = Comms::comms_layer.is_configured() ? 1 : 0;
    config_status_sendable.send_to_comms();

    if (false) { // Tests roundtrip comms latency. also needs to be set to true in hive.
        Comms::Sendable<TestLatencyData> latency_data;
        latency_data.data.current_time = micros();
        latency_data.data.time_since_last_received = micros() - Comms::comms_layer.get_hive_data().latency_data.current_time;
        latency_data.send_to_comms();
    }

    Comms::comms_layer.run();
    
}

void HelloRobot::check_safety() {
    float loop_dt = 0.0f;
    bool is_slow_loop = check_slow_loop(loop_dt);

    uint8_t previous_reasons = safety_state.active_reasons();

    uint8_t reasons = safety_state.evaluate(transmitter_manager.is_safety_mode(), Comms::comms_layer.is_configured(), is_slow_loop, ref.ref_data.robot_performance.gimbal_power_active);

    motors_armed = safety_state.motors_armed();

    if (motors_armed) {
        can.write();
    } else {
        // TODO: Reset all controller integrators here
        can.zero_all_motors();
        hold_feeder_position();
    }

    if (is_slow_loop) {
        SystemLog.error(Subsystem::GENERAL, "Slow loop with dt: %f, consecutive slow loops: %d\n", loop_dt, consecutive_slow_loops);
    }

    if (reasons != previous_reasons) {
        char reason_str[SafetyState::REASON_STR_LEN];
        SafetyState::reasons_to_string(reasons, reason_str, sizeof(reason_str));
        SystemLog.info(Subsystem::GENERAL, "Safety mode %s: %s\n", reasons ? "ON" : "OFF", reason_str);
    }
}

bool HelloRobot::check_slow_loop(float &loop_dt) {
    loop_dt = stall_timer.delta();
    if (loop_dt <= SLOW_LOOP_THRESHOLD_S) {
        consecutive_slow_loops = 0;
        return false;
    }

    consecutive_slow_loops++;

    if (consecutive_slow_loops > MAX_CONSECUTIVE_SLOW_LOOPS) {
        can.zero_all_motors();
        // reset_teensy never returns, so this path has to log for itself
        SystemLog.error(Subsystem::GENERAL, "Slow loop with dt: %f, consecutive slow loops: %d\n", loop_dt, consecutive_slow_loops);
        SystemLog.error("Kowabunga bitches\n");
        reset_teensy();
    }
    return true;
}

void HelloRobot::hold_feeder_position() {
    governor->hold_position(Cfg::StateName::Feeder, (*estimated_state_array)[Cfg::StateName::Feeder].get_position());
    if (has_lower_feeder) {
        governor->hold_position(Cfg::StateName::LowerFeeder, (*estimated_state_array)[Cfg::StateName::LowerFeeder].get_position());
    }

    Cfg::StateName fed_state = has_lower_feeder ? Cfg::StateName::LowerFeeder : Cfg::StateName::Feeder;
    float current_feed = (*estimated_state_array)[fed_state].get_position();

    // Snap the manual feed target to a whole ball so re-arming doesn't advance the feeder
    float whole_balls = floor(current_feed);
    feed = (current_feed - whole_balls > FEED_ROUND_UP_FRACTION) ? whole_balls + 1 : whole_balls;
    last_feed = feed;
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
