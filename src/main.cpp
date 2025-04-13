#include <Arduino.h>
#include <TeensyDebug.h>
#include "utils/timing.hpp"
#include "sensors/can/can_manager.hpp"
#include "sensors/can/MG8016EI6.hpp"
#include "sensors/ET16S.hpp"
#include "sensors/dr16.hpp"
#include "sensors/ICM20649.hpp"
#include "filters/IMU_filter.hpp"
#include "Controls_Balancing/test_balancing.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Speed scale will give the maximum desire speed (m/s)
#define SPEED_SCALE 4.5
// Speed scale will give the maximum desire rotation speed (rad/s)
#define ROTATION_SCALE 26 * M_1_PI //multiples of 1/pi radian/s = deg/s
#define MAX_LEG_LENGTH 0.33
#define MIN_LEG_LENGTH 0.18
// Declare global objects
CANManager can;
Timer loop_timer;
ICM20649 icm;
IMU_filter imu_filter;
balancing_test test_control;
Transmitter* transmitter = nullptr;
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
    uint32_t loopc = 0; // Loop counter for heartbeat
    pinMode(13, OUTPUT);
    Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);
    if (CrashReport) {
        while (1) {
            Serial.println(CrashReport);
            delay(1000);
        }
    }
    print_logo();

    TransmitterType transmitter_type = transmitter->who_am_i();
    if (transmitter_type == TransmitterType::DR16){
        transmitter = new DR16();
    }
    else if (transmitter_type == TransmitterType::ET16S){
        transmitter = new ET16S();
    }
    transmitter->init();
    
    test_control.init();
    can.init();
    SPI.begin(); // Start SPI for IMU
    icm.init(icm.SPI); // Initialize IMU 
    icm.set_gyro_range(4000); // Set gyro range to 4000 dps
    icm.calibration_all_w_angle_bias(); // Calibrate IMU
    // icm.calibration_all(); // Calibrate IMU
    imu_filter.init_EKF_6axis(icm.get_data()); // Initialize EKF filter
    
    // [controller_type, motor_id, bus_id]
    // controller_type: 
    // 0: invalid
    // 1: C610
    // 2: C620 
    // 3: MG8016EI6
    
    float motor_info[CAN_MAX_MOTORS][4] = {
        {3 , 1 , 2 , 0},
        {3 , 2 , 2 , 0},
        {3 , 3 , 2 , 0},
        {3 , 4 , 2 , 0},
        {6 , 5 , 2 , 0},
        {6 , 6 , 2 , 0},
    }; 
    can.configure(motor_info);
    // Reset all motor ROM
    // ((MG8016EI6*)can.get_motor(0))->write_motor_set_zero_ROM();
    // ((MG8016EI6*)can.get_motor(1))->write_motor_set_zero_ROM();
    // ((MG8016EI6*)can.get_motor(2))->write_motor_set_zero_ROM();
    // ((MG8016EI6*)can.get_motor(3))->write_motor_set_zero_ROM();
    // can.write();


    balancing_sensor_data data;
    // Main loop
    while (true) {
        // Read sensors
        transmitter->read();
        icm.read();
        icm.fix_raw_data(); // Fix the bias and scale factor
        can.read(); 
        
//---------------------------Controller code-----------------------------------
        imu_filter.step_EKF_6axis(icm.get_data());
        IMU_data* filtered_data = imu_filter.get_filter_data();
        data.gyro_roll = -filtered_data->gyro_pitch; // Roll is Y meaning the pitch from IMU filter -- right(+) 
        data.gyro_pitch = -filtered_data->gyro_roll; // Pitch is X meaning this is roll from IMU filter -- front(+)
        data.gyro_yaw = filtered_data->gyro_yaw; // Counterclockwise (+)
        data.imu_accel_x = abs(-filtered_data->accel_world_X) > 0.1 ? -filtered_data->accel_world_X : 0; // Front (+)
        data.imu_accel_y = filtered_data->accel_world_Y; // Right (+)
        data.imu_accel_z = -filtered_data->accel_world_Z; // up (+)
        data.imu_angle_pitch = -(filtered_data->roll - filtered_data->roll_bias);// Front(+)
        data.imu_angle_roll = -(filtered_data->pitch - filtered_data->pitch_bias);// Right(+)
        // data.imu_angle_pitch = -filtered_data->roll;// Front(+)
        // data.imu_angle_roll = -filtered_data->pitch;// Right(+)
        data.imu_angle_yaw = filtered_data->yaw; // CounterClockwise(+)
        data.angle_fr = can.get_motor(0)->get_state().position; // see from robot outor side clockwise (+) 
        data.angle_fl = can.get_motor(1)->get_state().position; // see from robot outor side clockwise (+)
        data.angle_bl = can.get_motor(2)->get_state().position; // see from robot outor side clockwise (+)
        data.angle_br = can.get_motor(3)->get_state().position; // see from robot outor side clockwise (+)
        data.speed_fr = can.get_motor(0)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_fl = can.get_motor(1)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_bl = can.get_motor(2)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_br = can.get_motor(3)->get_state().speed/ MG8016RATIO; // see from robot outor side clockwise (+)
        data.speed_wl = can.get_motor(4)->get_state().speed; // see from robot outor side clockwise (+)
        data.speed_wr = can.get_motor(5)->get_state().speed; // see from robot outor side clockwise (+)
        test_control.set_data(data); 
        test_control.observer();

        // float _dt = loop_timer.delta();
        if(transmitter->get_l_switch() == SwitchPos::MIDDLE){
            ref_data* control_ref = test_control.get_ref();
            // Set all data to 0 for balancing
            control_ref->s = 0;
            control_ref->s_dot = 0;
            control_ref->yaw = 0;
            control_ref->yaw_dot = 0;
            control_ref->theta_ll = 0;
            control_ref->theta_ll_dot = 0;
            control_ref->theta_lr = 0;
            control_ref->theta_lr_dot = 0;
            control_ref->pitch = 0;
            control_ref->pitch_dot = 0;
            
            control_ref->s_dot = transmitter->get_l_stick_y(); 
            if(abs(control_ref->s_dot) < 0.05){
                control_ref->s_dot = 0; // Ignore small data
            }else{
                control_ref->s_dot *= SPEED_SCALE; // Times a scale for max speed we set
                // control_ref->s += control_ref->speed * _dt;
                test_control.reset_s(); // Ignore the position when running 
            }
            control_ref->yaw_dot = -transmitter->get_l_stick_x(); 
            if(abs(control_ref->yaw_dot) < 0.05){
                control_ref->yaw_dot = 0; // Ignore small data
            }else{
                control_ref->yaw_dot *= ROTATION_SCALE; // Times a scale for max speed we set
                // control_ref->yaw += control_ref->yaw_dot * _dt;
                test_control.reset_yaw(); // Ignore yaw data when rotating
            }

            float leg_control = transmitter->get_r_stick_y() * 0.00007;
            if((leg_control > 0 && control_ref->goal_l < MAX_LEG_LENGTH) || (leg_control < 0 && control_ref->goal_l > MIN_LEG_LENGTH )){
                control_ref->goal_l += leg_control;
            }

            if (transmitter->get_r_stick_y() < -0.9) // reset safety mode after mulfunctioning is addressed
            {
                test_control.saftymode = 0;
            }
        }
        test_control.control();
        // can.write_motor_torque(0,test_control.getwrite().torque_fr);
        // can.write_motor_torque(1,test_control.getwrite().torque_fl);
        // can.write_motor_torque(2,test_control.getwrite().torque_bl);
        // can.write_motor_torque(3,test_control.getwrite().torque_br);
        can.write_motor_torque(4,test_control.getwrite().torque_wl);
        can.write_motor_torque(5,test_control.getwrite().torque_wr);

        // can.print_state();
        // test_control.print_observer();
        // test_control.printdata();
        test_control.print_visual();
        
        if (!transmitter->is_connected() || transmitter->get_l_switch() == SwitchPos::FORWARD || test_control.saftymode) {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            Serial.println("SAFTYON");
            can.issue_safety_mode();

            // reset s and yaw
            test_control.reset_s();
            test_control.reset_yaw();
        } else if (transmitter->is_connected() && transmitter->get_l_switch() != SwitchPos::FORWARD) {
            // SAFETY OFF
            Serial.println("SAFTYOFF");
            can.write();
            can.write();
        }
//---------------------------Debugging code-----------------------------------
        // ((MG8016EI6*)can.get_motor(0))->write_cmd_read_state_1();
        // ((MG8016EI6*)can.get_motor(1))->write_cmd_read_state_1();
        // ((MG8016EI6*)can.get_motor(2))->write_cmd_read_state_1();
        // ((MG8016EI6*)can.get_motor(3))->write_cmd_read_state_1();
        // can.write();
        // Serial.printf("waggle graph %s %f \n", "FR motor voltage", ((MG8016EI6*)can.get_motor(0))->get_voltage());
        // Serial.printf("waggle graph %s %f \n", "FL motor voltage", ((MG8016EI6*)can.get_motor(1))->get_voltage());
        // Serial.printf("waggle graph %s %f \n", "BL motor voltage", ((MG8016EI6*)can.get_motor(2))->get_voltage());
        // Serial.printf("waggle graph %s %f \n", "BR motor voltage", ((MG8016EI6*)can.get_motor(3))->get_voltage());

        // Serial.printf("waggle graph %s %f \n", "FR motor current", can.get_motor(0)->get_state().torque);
        // Serial.printf("waggle graph %s %f \n", "FL motor current", can.get_motor(1)->get_state().torque);
        // Serial.printf("waggle graph %s %f \n", "BL motor current", can.get_motor(2)->get_state().torque);
        // Serial.printf("waggle graph %s %f \n", "BR motor current", can.get_motor(3)->get_state().torque);
        // Keep the loop running at the desired rate
        // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;
        
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
    }
    return 0;
}
