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
    sensor_manager.init(config, &estimated_state_map_interrupt_safe);

    estimator_manager.init(config.estimators, sensor_manager, can);

    // generate controller outputs based on governed references and estimated
    // state
    controller_manager.init(config.controllers, can, config.states);

    estimated_state_map.emplace(config.states);
    estimated_state_map_interrupt_safe = std::make_unique<RobotStateMap>(config.states);
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

        read_telemetry();
        process_behaviors();
        update_controls();
        check_safety();
        loop_timing();
    }
}
void HelloRobot::crash_report() {
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

    // print loopc every second to verify it is still alive
    if (loopc % 1000 == 0) {
        Serial.println(loopc);
    }
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

    // showcase mode overrides both manual and hive targets
    if (transmitter_manager.is_showcase_mode()) {
        showcase_controls();
    }
    was_showcase_mode = transmitter_manager.is_showcase_mode();

    // override temp state if needed. Dont override in teensy mode so the sentry doesnt move during inspection
    if (Comms::comms_layer.get_hive_data().override_state_data.active && !(transmitter_manager.is_teensy_mode())) {
        // clear the request
        Comms::comms_layer.get_hive_data().override_state_data.active = false;

        hive_state_map_offset->from_comms_packet(Comms::comms_layer.get_hive_data().override_state_data.state);

        *estimated_state_map = *hive_state_map_offset;
        override_request = true;
    }
}
void HelloRobot::showcase_controls() {
    float dt = showcase_timer.delta();

    // on entry, start the yaw sweep from where the gimbal currently is so it doesn't jump
    if (!was_showcase_mode) {
        showcase_yaw = (*estimated_state_map)[Cfg::StateName::GimbalYaw].get_position();
        showcase_pitch_phase = 0;
        dt = 0;
    }

    // yaw: keep increasing, set_position wraps it into the configured range
    showcase_yaw += SHOWCASE_YAW_RATE * dt;

    // pitch: sine wave centered in the configured reference range
    showcase_pitch_phase += SHOWCASE_PITCH_RATE * dt;
    const Cfg::Limit& pitch_limits = (*estimated_state_map)[Cfg::StateName::GimbalPitch].config().reference_limits.position;
    float pitch_center = 0.5f * (pitch_limits.min + pitch_limits.max);
    float pitch_amplitude = 0.5f * SHOWCASE_PITCH_RANGE_FRACTION * (pitch_limits.max - pitch_limits.min);
    float pitch_target = pitch_center + pitch_amplitude * sinf(showcase_pitch_phase);

    (*target_state_map)[Cfg::StateName::GimbalYaw].set_position(showcase_yaw);
    (*target_state_map)[Cfg::StateName::GimbalYaw].set_velocity(0);
    (*target_state_map)[Cfg::StateName::GimbalPitch].set_position(pitch_target);
    (*target_state_map)[Cfg::StateName::GimbalPitch].set_velocity(0);

    // everything dangerous is told to stop (and its motors are zeroed in check_safety regardless)
    (*target_state_map)[Cfg::StateName::ChassisX].set_position((*estimated_state_map)[Cfg::StateName::ChassisX].get_position());
    (*target_state_map)[Cfg::StateName::ChassisX].set_velocity(0);
    (*target_state_map)[Cfg::StateName::ChassisY].set_position((*estimated_state_map)[Cfg::StateName::ChassisY].get_position());
    (*target_state_map)[Cfg::StateName::ChassisY].set_velocity(0);
    (*target_state_map)[Cfg::StateName::ChassisHeading].set_velocity(0);
    (*target_state_map)[Cfg::StateName::Flywheels].set_velocity(0);
    (*target_state_map)[Cfg::StateName::Feeder].set_position((*estimated_state_map)[Cfg::StateName::Feeder].get_position());
    (*target_state_map)[Cfg::StateName::Feeder].set_velocity(0);
    if (has_lower_feeder) {
        (*target_state_map)[Cfg::StateName::LowerFeeder].set_position((*estimated_state_map)[Cfg::StateName::LowerFeeder].get_position());
        (*target_state_map)[Cfg::StateName::LowerFeeder].set_velocity(0);
    }
}

void HelloRobot::update_controls() {
    // step estimates and construct estimated state
    estimator_manager.step(*estimated_state_map, override_request);
    // estimated_state_map.print();

    noInterrupts();
    *estimated_state_map_interrupt_safe = *estimated_state_map;
    interrupts();
    
    override_request = false;
    float current_feed = (*estimated_state_map)[Cfg::StateName::Feeder].get_position();
    float target_feed = (*target_state_map)[Cfg::StateName::Feeder].get_position();
    if ((feed - current_feed > 2 && transmitter_manager.is_teensy_mode()) || (target_feed - current_feed > 2 && transmitter_manager.is_hive_mode())) {
        Serial.printf("Feeder is lowkey jammed. current ball count: %f, feed: %f, hive target: %f\n", (*estimated_state_map)[Cfg::StateName::Feeder].get_position(), feed, (*target_state_map)[Cfg::StateName::Feeder].get_position());
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
    controller_manager.step(*reference_map, *estimated_state_map, *target_state_map);

    target_state_map->send_to_comms<TargetState>();
    reference_map->send_to_comms<ReferenceState>();
    estimated_state_map->send_to_comms<EstimatedState>();

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
    bool is_slow_loop = false;

    // check whether this was a slow loop or not
    float dt = stall_timer.delta();
    if (dt > 0.002f) {
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

    not_safety_mode = (!transmitter_manager.is_safety_mode() && Comms::comms_layer.is_configured() && !is_slow_loop && ref.ref_data.robot_performance.gimbal_power_active && !gimbal_power_recently_turned_on);

    safety::set_safety_mode(!not_safety_mode);

    //  SAFETY MODE
    if (not_safety_mode) {
        // SAFETY OFF
        if (transmitter_manager.is_showcase_mode()) {
            // only the gimbal is allowed to move in showcase mode
            can.zero_non_gimbal_motors();
        }
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
        feed = (fmod(fmod(current_feed, 1) + 1, 1) > 0.2)
                   ? (int)floor(current_feed) + 1
                   : (int)floor(current_feed); // reset feed to the current state
        last_feed = feed;                      // reset last feed to the current state
                                               // Serial.printf("Can zero\n");
    }
}
void HelloRobot::loop_timing() {
    // LED heartbeat -- linked to loop count to reveal slowdowns and
    // freezes.
    loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
    loopc++;

    // feed the watchdog to keep the loop running
    watchdog.feed();

    // Keep the loop running at the desired rate
    loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
}
