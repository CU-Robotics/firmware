#include <Arduino.h>

#include "git_info.h"

#include "utils/profiler.hpp"
#include "utils/timing.hpp"
#include <TeensyDebug.h>

#include "sensors/can/C610.hpp"
#include "sensors/can/C620.hpp"
#include "sensors/can/MG8016EI6.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

Profiler prof;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_2;

Timer loop_timer;

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
    long long loopc = 0; // Loop counter for heartbeat

    Serial.begin(115200); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);

    print_logo();

    can_2.begin();
    can_2.setBaudRate(1000000);

    CAN_message_t output_msg;

    MG8016EI6 motor(MotorType::MG8016E_I6V3, 0, 1, 2, &can_2);
    
    motor.write_motor_angle(0);
    motor.write(output_msg);

    // Execute setup functions
    pinMode(13, OUTPUT);

    // Main loop
    while (true) {
        if (loopc % 500 == 0) {
            Serial.printf("Alive %ld\n", loopc);
        }
        
        motor.print_state();
        can_2.write(output_msg);

        CAN_message_t read_msg;
        if (can_2.read(read_msg)) {
            motor.read(read_msg);
        }


        // if (loopc > 5000) {
        //     motor.write_motor_torque(0);
        //     motor.write(output_msg);
        // }
        
        // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
    }
    
    return 0;
}


