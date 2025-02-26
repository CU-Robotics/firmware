#include <Arduino.h>
#include <TeensyDebug.h>
#include "utils/timing.hpp"
#include "sensors/can/can_manager.hpp"
#include "sensors/can/MG8016EI6.hpp"
#include "sensors/dr16.hpp"
#include "sensors/ICM20649.hpp"
#include "filters/IMU_filter.hpp"
#include "Controls_Balancing/test_balancing.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2
// Declare global objects
DR16 dr16;
CANManager can;
Timer loop_timer;
ICM20649 icm;
IMU_filter imu_filter;
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
    can.init();
    SPI.begin(); // Start SPI for IMU
    icm.init(icm.SPI); // Initialize IMU 
    icm.set_gyro_range(4000); // Set gyro range to 4000 dps
    icm.calibration_all(); // Calibrate IMU
    imu_filter.init_EKF_6axis(icm.get_data()); // Initialize EKF filter
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
    // Main loop
    while (true) {
        // Read sensors
        dr16.read();
        icm.read();
        icm.fix_raw_data(); // Fix the bias and scale factor
        can.read();

        imu_filter.step_EKF_6axis(icm.get_data());
        IMU_data* filtered_data = imu_filter.get_filter_data();
        data.gyro_roll = -filtered_data->gyro_pitch; // Roll is Y meaning the pitch from IMU filter -- right(+) 
        data.gyro_pitch = filtered_data->gyro_roll; // Pitch is X meaning this is roll from IMU filter -- front(+)
        data.gyro_yaw = filtered_data->gyro_yaw; // Counterclockwise (+)
        data.imu_accel_x = -filtered_data->accel_world_X; // Front (+)
        data.imu_accel_y = filtered_data->accel_world_Y; // Right (+)
        data.imu_accel_z = -filtered_data->accel_world_Z; // up (+)
        data.imu_angle_pitch = -filtered_data->roll;  // Front(+)
        data.imu_angle_roll = -filtered_data->pitch; // Right(+)
        data.imu_angle_yaw = filtered_data->yaw; // CounterClockwise(+)
        data.angle_fr = can.get_motor(0)->get_state().position; // see from robot outor side clockwise (+) 
        data.angle_fl = can.get_motor(1)->get_state().position; // see from robot outor side clockwise (+)
        data.angle_bl = can.get_motor(2)->get_state().position; // see from robot outor side clockwise (+)
        data.angle_br = can.get_motor(3)->get_state().position; // see from robot outor side clockwise (+)
        data.speed_fr = can.get_motor(0)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_fl = can.get_motor(1)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_bl = can.get_motor(2)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_br = can.get_motor(3)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_wl = can.get_motor(4)->get_state().speed/ M3508RATIO; // see from robot outor side clockwise (+)
        data.speed_wr = can.get_motor(5)->get_state().speed/ M3508RATIO; // see from robot outor side clockwise (+)
        test_control.set_data(data); 
        test_control.observer();
        test_control.control();
        can.write_motor_torque(0,test_control.getwrite().torque_fr);
        can.write_motor_torque(1,test_control.getwrite().torque_fl);
        can.write_motor_torque(2,test_control.getwrite().torque_bl);
        can.write_motor_torque(3,test_control.getwrite().torque_br);
        can.write_motor_torque(4,test_control.getwrite().torque_wl);
        can.write_motor_torque(5,test_control.getwrite().torque_wr);

        // can.print_state();
        test_control.print_observer();
        test_control.printdata();
        
        if (!dr16.is_connected() || dr16.get_l_switch() == 1 || test_control.saftymode) {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            Serial.println("SAFTYON");
            can.issue_safety_mode();
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
