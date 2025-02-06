#include <Arduino.h>

#include "git_info.h"

#include "utils/profiler.hpp"
#include "sensors/d200.hpp"
#include "sensors/StereoCamTrigger.hpp"
#include "controls/estimator_manager.hpp"
#include "controls/controller_manager.hpp"

#include <TeensyDebug.h>
#include "sensors/LEDBoard.hpp"
#include "data_packet.hpp"
#include "sensor_constants.hpp"

    // Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Declare global objects
DR16 dr16;
CANManager can;
RefSystem ref;
HIDLayer comms;
ACS712 current_sensor;

D200LD14P lidar1(&Serial4, 0);
D200LD14P lidar2(&Serial5, 1);

StereoCamTrigger stereoCamTrigger(60);

ConfigLayer config_layer;

Profiler prof;

Timer loop_timer;
Timer stall_timer;
Timer control_input_timer;

EstimatorManager estimator_manager;
ControllerManager controller_manager;

Governor governor;

LEDBoard led;

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
        Serial.printf("\nGit Hash: %s", GIT_COMMIT_HASH);
        Serial.printf("\nGit Branch: %s", GIT_BRANCH);
        Serial.printf("\nCommit Message: %s", GIT_COMMIT_MSG);
        Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}

// Master loop
int main() {
    Serial.begin(112500); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);

    print_logo();

    // Execute setup functions
    pinMode(LED_BUILTIN, OUTPUT);

    can.init();

    // CTRL type, motor ID, bus ID
    // CTRL type: 0 = none, 1 = C610, 2 = C620, 3 = MG8016
    float motor_info[24][3] = {
        {2, 1, 1},
    };

    can.configure(motor_info);

    Serial.println("Entering main loop...\n");

    // Main loop
    while (true) {
        // read main sensors
        can.read();

        can.write_motor_torque(CAN_2, 1, 0.1);

        can.write();

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
    }

    return 0;
}