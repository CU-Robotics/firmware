#include "teleop_manager.hpp"

TeleopManager::TeleopManager() {
    control_input_timer.start();
}

void TeleopManager::process_manual_controls(
        Transmitter* tx, 
        const RefSystem& ref, 
        const RobotStateMap& estimated_state, 
        RobotStateMap& target_state, 
        float& feed, 
        float& last_feed) {
        
    // ==========================================
    // 1. GET PHYSICAL INTENT (Zero Allocations)
    // ==========================================
    // Pass our persistent struct down to the radio to be overwritten
    tx->gete_intent(current_intent);

    // ==========================================
    // 2. ARBITRATION & VTM SUPERPOSITION
    // ==========================================
    float delta = control_input_timer.delta();


	// Superimpose VTM WASD on top of the stick intent
	current_intent.chassis_vel_x += (-ref.ref_data.kbm_interaction.key_w + ref.ref_data.kbm_interaction.key_s) * 2.5f;
	current_intent.chassis_vel_y += (ref.ref_data.kbm_interaction.key_d - ref.ref_data.kbm_interaction.key_a) * 2.5f;

	// Superimpose VTM Mouse on top of the stick intent
	vtm_yaw_offset -= ref.ref_data.kbm_interaction.mouse_speed_x * 0.05f * delta;
	vtm_pitch_offset += ref.ref_data.kbm_interaction.mouse_speed_y * 0.05f * delta;
        
	current_intent.yaw_target += vtm_yaw_offset;
	current_intent.pitch_target += vtm_pitch_offset;

	// VTM Mouse Clicks map to the Dual Feeders
	if (ref.ref_data.kbm_interaction.button_left) {
		current_intent.top_feeder_target = 10.0f;
	}
	if (ref.ref_data.kbm_interaction.button_right) {
		current_intent.bottom_feeder_target = 10.0f;
	}


    // ==========================================
    // 3. CAPABILITY MAPPING & KINEMATICS
    // ==========================================
    
    // Apply Universal Systems
    target_state[Cfg::StateName::ChassisX].set_velocity(current_intent.chassis_vel_x);
    target_state[Cfg::StateName::ChassisY].set_velocity(current_intent.chassis_vel_y);
    target_state[Cfg::StateName::ChassisHeading].set_velocity(current_intent.chassis_spin);
    
    target_state[Cfg::StateName::GimbalPitch].set_position(current_intent.pitch_target);
    target_state[Cfg::StateName::GimbalYaw].set_position(current_intent.yaw_target);

    // Apply Legacy Robot (Single Feeder)
    if (target_state.contains(Cfg::StateName::Feeder)) {
        target_state[Cfg::StateName::Feeder].set_velocity(current_intent.top_feeder_target);
    }

    // Apply New Robot (Dual Feeders)
    if (target_state.contains(Cfg::StateName::FeederTop)) {
        target_state[Cfg::StateName::FeederTop].set_velocity(current_intent.top_feeder_target);
    }
    if (target_state.contains(Cfg::StateName::FeederBottom)) {
        target_state[Cfg::StateName::FeederBottom].set_velocity(current_intent.bottom_feeder_target);
    }

    // ==========================================
    // 4. GLOBAL MODE TRACKING
    // ==========================================
    // If the pilot toggles a switch to change modes, ensure the feeders don't 
    // suddenly jump by resetting the current feed target to the last known safe feed.
    if (tx->is_teensy_mode() && tx->mode_changed()) {
        feed = last_feed;
    }
}
