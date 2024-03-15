#include <Arduino.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "comms/usb_hid.hpp"
// Loop constants
#define LOOP_FREQ      1000
#define HEARTBEAT_FREQ 2

// Declare global objects
DR16 dr16;
//rm_CAN can;
Timer loop_timer;
HIDLayer comms;

// DONT put anything else in this function. It is not a setup function
void print_logo() {
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
		Serial.println("\n\033[1;92mFW Ver. 2.1.0");
		Serial.printf("\nLast Built: %s at %s", __DATE__, __TIME__);
		Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
		Serial.println("\033[0m\n");
	}
}

// Master loop
int main() {
	Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
	print_logo();

	// Execute setup functions
	pinMode(13, OUTPUT);
	dr16.init();
	//can.init();
	comms.init();

	float state[STATE_LEN][3];
	for (int i = 0; i < STATE_LEN; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			state[i][j] = i;
		}
	}

	long long loopc = 0; // Loop counter for heartbeat
	
	// Main loop
	while (true) {
		// Read sensors
		dr16.read();
		//can.read();
	
		comms.get_outgoing()->set_time((double)millis());
		comms.get_outgoing()->set_estimated_state(state);
		SensorData sensor_data;
		memcpy(sensor_data.raw, dr16.get_raw(), DR16_PACKET_SIZE);
		comms.get_outgoing()->set_sensor_data(&sensor_data);

		comms.ping();

        // Write actuators
        if (dr16.is_connected() && (dr16.get_l_switch() == 2 || dr16.get_l_switch() == 3)) {
        // SAFETY OFF
            can.write();
        } else {
             // SAFETY ON
             // TODO: Reset all controller integrators here
            can.zero();
        }

		// Keep the loop running at the desired rate
		// loop_timer.delay_micros((int)(1E6/(float)(LOOP_FREQ)));
	}

	return 0;
}
