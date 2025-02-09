#include <Arduino.h>
#include <TeensyDebug.h>
#include "utils/timing.hpp"
#include "comms/can/can_manager.hpp"
#include "comms/can/MG8016EI6.hpp"
#include "sensors/dr16.hpp"
#include "filters/IMU_Filter.hpp"
#include "Controls_Balancing/test_balancing.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2
// Declare global objects
DR16 dr16;
CANManager can;
Timer loop_timer;
IMU_filter icm;
balancing_test test_control;
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
    test_control.init();
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
        {2 , 1 , 2},
        {2 , 3 , 2}
    }; 
    can.configure(motor_info);
    // ((MG8016EI6*)can.get_motor(0))->write_motor_set_zero_ROM();
    // ((MG8016EI6*)can.get_motor(1))->write_motor_set_zero_ROM();
    // ((MG8016EI6*)can.get_motor(2))->write_motor_set_zero_ROM();
    // ((MG8016EI6*)can.get_motor(3))->write_motor_set_zero_ROM();
    // can.write();


    balancing_sensor_data data;
    IMUData imu_data;
    // Main loop
    while (true) {
        // Read sensors
        dr16.read();
        icm.read();
        can.read();

        imu_data = icm.getdata();
        data.gyro_pitch = imu_data.alpha_pitch;
        data.gyro_roll = imu_data.alpha_roll;
        data.gyro_yew = imu_data.alpha_yaw;
        data.imu_accel_x = imu_data.world_accel_X;
        data.imu_accel_y = imu_data.world_accel_Y;
        data.imu_accel_z = imu_data.world_accel_Z;
        data.imu_angle_pitch = imu_data.k_pitch;
        data.imu_angle_roll = imu_data.k_roll;
        data.angle_fr = can.get_motor(0)->get_state().position;
        data.angle_fl = can.get_motor(1)->get_state().position;
        data.angle_bl = can.get_motor(2)->get_state().position;
        data.angle_br = can.get_motor(3)->get_state().position;
        data.speed_fr = can.get_motor(0)->get_state().speed;
        data.speed_fl = can.get_motor(1)->get_state().speed;
        data.speed_bl = can.get_motor(2)->get_state().speed;
        data.speed_br = can.get_motor(3)->get_state().speed;
        data.speed_wl = can.get_motor(4)->get_state().speed;
        data.speed_wr = can.get_motor(5)->get_state().speed;
        
        test_control.set_data(data);
        test_control.observer();
        test_control.control_position();
        can.write_motor_torque(0,test_control.getwrite().torque_fr);
        can.write_motor_torque(1,test_control.getwrite().torque_fl);
        can.write_motor_torque(2,test_control.getwrite().torque_bl);
        can.write_motor_torque(3,test_control.getwrite().torque_br);
        can.write_motor_torque(4,test_control.getwrite().torque_wl);
        can.write_motor_torque(5,test_control.getwrite().torque_wr);

        // can.print_state();
        test_control.print_observer();
        test_control.printdata();
        // icm.print();
        if (!dr16.is_connected() || dr16.get_l_switch() == 1 || test_control.saftymode) {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            Serial.println("SAFTYON");
            can.safety_mode();
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
