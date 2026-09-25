#include "robot_cli.hpp"

void RobotCLI::init(
    TransmitterManager& tx,
    SensorManager& sensors,
    const RobotStateArray& estimated_state,
    const RobotStateArray& target_state,
    const uint32_t& loop_counter
) {
    transmitter_manager = &tx;
    sensor_manager = &sensors;
    estimated_state_array = &estimated_state;
    target_state_array = &target_state;
    loopc = &loop_counter;

    SystemLog.bind_cli_buffer(cli_buffer);
}

void RobotCLI::process() {
    if (num_active_views > 0) {
        render_live_view();
    } else {
        parse_serial_stream();
    }
}

void RobotCLI::render_live_view() {
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
				transmitter_manager->print_live_data();
				break;
                        
			case LiveMode::SENSORS:
				Serial.printf("=== LIVE SENSOR READOUT ===\033[K\n");
				sensor_manager->print_sensors_live(); 
				break;
                        
			case LiveMode::ESTIMATED_STATE:
				Serial.printf("=== LIVE ESTIMATED STATE ===\033[K\n");
				estimated_state_array->print();
				break;
				
			case LiveMode::TARGET_STATE:
				Serial.printf("=== LIVE TARGET STATE ===\033[K\n");
				target_state_array->print();
				break;

			case LiveMode::HEARTBEAT:
				Serial.printf("=== LIVE HEARTBEAT  ===\033[K\n");
				if (loopc)Serial.println(*loopc);
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
    
}
void RobotCLI::parse_serial_stream() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        static bool last_was_cr = false;
        // Handle Backspace (ASCII 8 or DEL 127)
        if (c == '\n' && last_was_cr) {
            last_was_cr = false;
            continue; 
        }
        last_was_cr = (c == '\r');
        if (c == '\b' || c == 127) {
            if (cli_index > 0) {
                cli_index--;
                cli_buffer[cli_index] = '\0';
                
                // \r moves to the start of the line, \033[K erases it, then we redraw
                Serial.print("\r\033[KRobot> ");
                Serial.print(cli_buffer);
            }
            continue;
        }
        if (c == '\n' || c == '\r') {
			if (cli_index == 0) {
                Serial.print("\r\nRobot> "); // Reprint prompt on empty enter
                continue; 
            }

            Serial.println(); // Move to new line so command output doesn't overwrite the prompt
            cli_buffer[cli_index] = '\0';

            // --- THE COMMAND DICTIONARY ---
            static const struct {
                const char* name;
                void (RobotCLI::*execute)();
            } commands[] = {
                {"ping", &RobotCLI::cmd_ping},
                {"help", &RobotCLI::cmd_help},
                {"live", &RobotCLI::cmd_live},
                {"log", &RobotCLI::cmd_log}
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
            cli_buffer[0] = '\0';
            if (num_active_views == 0) {
                Serial.print("\r\nRobot> ");
            }
        } 
        else if (cli_index < 63) {
			cli_buffer[cli_index++] = c;
            cli_buffer[cli_index] = '\0'; // keep it null-terminated
            
            // Clear the line and  redraw the buffer on every keystroke
            Serial.print("\r\033[KRobot> ");
            Serial.print(cli_buffer);
        }
    } 
}
void RobotCLI::cmd_ping() {
    Serial.println("pong! Robot is alive.");
}

void RobotCLI::cmd_help() {
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
    Serial.println("                estimated_state : The robot's current estimated state array");
    Serial.println("                target_state    : The robot's current target state array");
    Serial.println("                heartbeat       : The main loop counter (loopc)");
	Serial.println();
	Serial.println("       log [subsystem] [priority]");
	Serial.println("              Filters the system event log.");
	Serial.println("              High-priority messages (Errors) will always bypass the subsystem filter.");
	Serial.println("              Typing 'log' with no arguments displays the syntax menu.");
	Serial.println();
	Serial.println("              Available subsystems:");
	Serial.println("                all, can, motors, sensors, est, comms");
	Serial.println();
	Serial.println("              Available priorities (minimum level to show):");
	Serial.println("                info, warn, error");
	Serial.println();
	Serial.println("              Examples:");
	Serial.println("                log motors warn  : Shows motor warnings/errors, and all other system errors");
	Serial.println("                log all info     : Resets the filter to show absolutely everything");
    Serial.println();
    Serial.println("       help");
    Serial.println("              Displays this manual.");
}
void RobotCLI::cmd_live() {
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
        SystemLog.is_live_view_active = false;
        Serial.println("Usage: live [prof] [tx] [sensors] [estimated_state] [target_state] [heartbeat]");
    }
}

void RobotCLI::cmd_log() {
    char* sys_tok = strtok(NULL, " ");
    char* lvl_tok = strtok(NULL, " ");

    // --- DATA DICTIONARIES ---
    struct SysMap {
        const char* name;
        Subsystem sys;
    };
    static const SysMap sys_dict[] = {
        {"all",     Subsystem::ALL},
        {"can",     Subsystem::CAN},
        {"motors",  Subsystem::MOTORS},
        {"sensors", Subsystem::SENSORS},
        {"est",     Subsystem::ESTIMATOR},
        {"comms",   Subsystem::COMMS}
    };

    struct LvlMap {
        const char* name;
        LogLevel lvl;
    };
    static const LvlMap lvl_dict[] = {
        {"info",  LogLevel::INFO},
        {"warn",  LogLevel::WARN},
        {"error", LogLevel::ERROR}
    };

    bool error_found = false;

    // Check Subsystem (if provided)
    if (sys_tok) {
        bool found = false;
        for (const auto& entry : sys_dict) {
            if (strcmp(sys_tok, entry.name) == 0) {
                SystemLog.view_filter_sys = entry.sys;
                found = true;
                break;
            }
        }
        
        if (!found) {
            Serial.printf("Error: Unknown subsystem '%s'\n", sys_tok);
            error_found = true;
        }
    }

    // Check Priority Level (if provided)
    if (lvl_tok && !error_found) {
        bool found = false;
        for (const auto& entry : lvl_dict) {
            if (strcmp(lvl_tok, entry.name) == 0) {
                SystemLog.view_filter_level = entry.lvl;
                found = true;
                break;
            }
        }
        
        if (!found) {
            Serial.printf("Error: Unknown priority level '%s'\n", lvl_tok);
            error_found = true;
        }
    }

    // Check if log statement was written correctly
    if (error_found || (!sys_tok && !lvl_tok)) {
        Serial.println("Usage: log [subsystem] [priority]");
        Serial.println("  Subsystems: all, can, motors, sensors, est, comms");
        Serial.println("  Priorities: info, warn, error");
        Serial.println("  Example:    log motors warn");
        return; 
    }

    Serial.printf("Log filter updated. Sys: %s | Level: %s\n", 
				  sys_tok ? sys_tok : "UNCHANGED", 
				  lvl_tok ? lvl_tok : "UNCHANGED");
}
