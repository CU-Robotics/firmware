#include "utils/timing.h"
#include "sensors/dr16.hpp"
#include "comms/rm_CAN.hpp"

uint32_t cycle_time_us = 1000;
uint32_t cycle_time_ms = cycle_time_us / 1000;
float cycle_time_s = cycle_time_us * 1E-6;

DR16 dr16;
rm_CAN can;

// Runs once
void setup() {
	Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)

    dr16.init(); // set up dr16
    can.init(); // set up can

	if (Serial) {
		Serial.println("TEENSY SERIAL START\n\n");
        Serial.print("\033[1;33m");
        Serial.println("                  .:^!?!^.                        ");
        Serial.println("           .:~!?JYYYJ?7?Y5Y7!!.                   ");
        Serial.println("         :?5YJ?!~:.      ^777YP?.                 ");
        Serial.println("         5G~                  ~YP?:               ");
        Serial.println("         7P5555Y:               ^YP?:....         ");
        Serial.println("        ~55J7~^.   ..    .        ^JYYYYYYYYYJJ!. ");
        Serial.println("        YG^     !Y5555J:^PJ    Y5:      ...::^5G^ ");
        Serial.println("       :GY    .YG?^..^~ ~GY    5G^ ^!~~^^^!!~7G?  ");
        Serial.println(" .!JYYY5G!    7BJ       ~GY    5G^ ~??JJJY555GP!  ");
        Serial.println("^55!^:.^~.    ^PP~   .: ^GP:  ^PP:           :7PY.");
        Serial.println("YG^            :JP5YY55: ~YP55PY^              ~GJ");
        Serial.println("?G~      .?7~:   .^~~^.    .^:.                :G5");
        Serial.println(".5P^     7BYJ5YJ7^.                          .~5P^");
        Serial.println(" .JPJ!~!JP?  .:~?PP^            .:.    .^!JYY5Y!. ");
        Serial.println("   :!???!:       5P.         .!Y5YYYJ?Y5Y?!^:.    ");
        Serial.println("                 7G7        7GY!. .:~!^.          ");
        Serial.println("                  JG!      :G5                    ");
        Serial.println("                   7PY!^^~?PY:                    ");
        Serial.println("                    .!JJJJ?^                      ");
        Serial.print("\033[0m");
		Serial.println("\n\033[1;92mFW Ver. 2.0.0");
        Serial.print("\nBoot datetime: 00/00/00 at 00:00am");
		Serial.print("Boot hash (random): 0x");
		unsigned long time = micros();
		Serial.print(time & 0xFFFF, HEX);
		Serial.println("\033[0m\n");
	}
}

// Master loop
int main() { // Basically a schudeling algorithm
    dr16.read(); // read data from controller

    Timer timer;

	unsigned long prev_time = micros();

	while (true) {
		timer.startTimer()

		// Calculate dt
		unsigned long curr_time = micros();
		float dt = (curr_time - prev_time) / 1000000.0;
		prev_time = curr_time;

		timer.delayMicros(0, cycle_time_us);	        // normalize master loop cycle time to cycle_time_u
		blink();										// helpful if you think the loop is crashing (light will pause)
	}

    // if info from the remote is not being detected
    // or if safety switch is on, don't write anything
    if (!dr16.is_connected() || dr16.get_l_switch() == 1) {
        Serial.println("SAFETY: ON");

        can.zero();
        can.zero_motors();
    } else {
        Serial.println("SAFETY: OFF");
        
        while (can.read()) {}

        // control code goes here

        can.write();
    }

	return 0;
}
