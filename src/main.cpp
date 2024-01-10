#include <Arduino.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "filters/pid_filter.hpp"

// declare any 'global' variables here
DR16 dr16;
rm_CAN can;

Timer loop_timer;

// REMOVE THIS
PIDFilter fortnite;
PIDFilter roblox1;
PIDFilter roblox2;

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

    // initialize any 'setup' functions here
    pinMode(13, OUTPUT);
    dr16.init();
    can.init();

    fortnite.K[0] = 0.0125;
    fortnite.K[2] = 0.005;
    roblox1.K[0] = 0.0008;
    roblox2.K[0] = 0.0008;

    // main loop
    while (true) {
        dr16.read();
        can.read();
        if (!dr16.is_connected() || dr16.get_l_switch() == 1) {
            // SAFETY ON
            can.zero();
            can.zero_motors();
        } else if (dr16.is_connected() && dr16.get_l_switch() != 1) {
            // SAFETY OFF

            // REMOVE THIS
            float motor_speed = can.get_motor_attribute(CAN_2, 4, MotorAttribute::SPEED) / 36.0;
            fortnite.setpoint = dr16.get_l_stick_x() * 200;
            fortnite.measurement = motor_speed;
            float output = fortnite.filter(0.001);
            can.write_motor_norm(CAN_2, 4, C610, output);

            ///

            motor_speed = can.get_motor_attribute(CAN_2, 2, MotorAttribute::SPEED);
            roblox1.setpoint = -(dr16.get_r_switch()-1) * 4500;
            roblox1.measurement = motor_speed;
            output = roblox1.filter(0.001);
            can.write_motor_norm(CAN_2, 2, C620, output);

            ///

            motor_speed = can.get_motor_attribute(CAN_2, 3, MotorAttribute::SPEED);
            roblox2.setpoint = (dr16.get_r_switch()-1) * 4500;
            roblox2.measurement = motor_speed;
            output = roblox2.filter(0.001);
            can.write_motor_norm(CAN_2, 3, C620, output);

            can.write_motor_norm(CAN_2, 5, C610, dr16.get_r_stick_x());

            can.write();
        }

        // LED heartbeat
        millis() % 500 < 100 ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);

        // Keep the loop running at 1kHz
        loop_timer.delay_millis(1);
    }

    return 0;
}
