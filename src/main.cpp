#include <Arduino.h>
#include <TeensyDebug.h>
#include "utils/timing.hpp"
#include "comms/can/C610.hpp"
#include "comms/can/C620.hpp"
#include "comms/can/MG8016EI6.hpp"
#include "comms/can/can_manager.hpp"
#include "sensors/dr16.hpp"
#include "filters/IMU_Filter.hpp"
#include "Controls_Balancing/Balancing_Control.hpp"
#include "Controls_Balancing/Balancing_Observer.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2
// Declare global objects
DR16 dr16;
CANManager can;
Timer loop_timer;
IMU_filter icm;
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
int main() {
    Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);
    
    if (CrashReport) {
        while (1) {
            Serial.println(CrashReport);
            delay(1000);
        }
    }

    print_logo();
    dr16.init();
    
    icm.init();

    can.init();

    // [controller_type, motor_id, bus_id]
    // controller_type: 
    // 0: invalid
    // 1: C610
    // 2: C620 
    // 3: MG8016EI6
    
    float motor_info[CAN_MAX_MOTORS][3] = {
        {3 , 1 , 2},
        {3 , 2 , 2},
        {3 , 3 , 2},
        {3 , 4 , 2},
        {2 , 1 , 0},
        {2 , 2 , 0}
    }; 
    can.configure(motor_info);
    
    // Main loop
    while (true) {
        // Read sensors
        dr16.read();
        icm.read();


        //can.write();

        can.read();
        can.print_state();

        //Testobserver.step(can.get_data(), imu.getdata(), tempobs); // Calculate Observer values
        //Testcontorl.step(tempmotor, ref, tempobs); // Calculate motors motion
        icm.print();


        //** Temp limit functions -- Will be put inside controller when using*/
        // Basically I use 90 degree as a limit
        // so it will be angle A - B <= 90 degree(Need test)
        // Also the A and B are limited to their limit (Need test)


        //testicm.print();
        // Write actuators
        if (!dr16.is_connected() || dr16.get_l_switch() == 1) {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.safety_mode();
            Serial.println("SAFTYON");
        } else if (dr16.is_connected() && dr16.get_l_switch() != 1) {
            // SAFETY OFF
            Serial.println("SAFTYOFF");
            can.write();
        }


        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
    }
    return 0;
}