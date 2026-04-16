#pragma once

struct RobotIntent {
    float chassis_vel_x = 0;
    float chassis_vel_y = 0;
    float chassis_spin = 0;
    float pitch_target = 0;
    float yaw_target = 0;
    float flywheel_target = 0;
    float feeder_target = 0;

    bool native_kbm_active = false; 
};

class TeleopManager(){
	void manual_controls(Transmitter* tx, 
						 const RefSystem& ref, 
						 const RobotStateMap& estimated_state, 
						 RobotStateMap& target_state,
						 bool not_safety_mode,
						 float& feed,
						 float& last_feed);


}
