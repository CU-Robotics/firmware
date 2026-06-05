#include "hello_robot.hpp"

// Master loop
int main() {

    Serial.begin(115200); // the serial monitor is actually always active (for
                          // debug use Serial.println & tycmd)
	while(!Serial);
    debug.begin(SerialUSB1);

    Utils::print_logo();

    static HelloRobot robot;
    robot.init();
    robot.run();

    return 0;
}
