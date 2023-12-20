#include "utils/timing.h"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "comms/usb_hid.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/LSM6DSOX.hpp"



// Runs once
void setup()
{
    Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)

    if (Serial)
    {
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
        Serial.printf("\nLast Built: %s at %s", __DATE__, __TIME__);
        Serial.printf("\nRun Hash:   %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}

// Master loop
int main()
{ // Basically a schudeling algorithm
    DR16 dr16;
    rm_CAN can;

    setup();

    dr16.init();
    can.init();

    if (!dr16.is_connected() || dr16.get_l_switch() == 1)
    {
        Serial.println("SAFETY: ON");

        can.zero();
        can.zero_motors();
    }
    else
    {
        Serial.println("SAFETY: OFF");

        can.read();

        // control code goes here

        can.write();
    }

    return 0;
}
