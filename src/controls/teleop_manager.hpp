#pragma once

class TeleopManager(){
	void manual_controls(Transmitter* tx, 
						 const RefSystem& ref, 
						 const RobotStateMap& estimated_state, 
						 RobotStateMap& target_state,
						 bool not_safety_mode,
						 float& feed,
						 float& last_feed);
	
	


}
