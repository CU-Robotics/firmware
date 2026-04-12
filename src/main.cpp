#include "hello_robot.hpp"


//extern "C" void reset_teensy(void);

// Master loop
int main() {

    Serial.begin(115200); // the serial monitor is actually always active (for
                          // debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);
	
	//Print Splash Screen
	Utils::print_logo();
	static HelloRobot robot;
    robot.init();
    robot.run();
	
    return 0;
}
