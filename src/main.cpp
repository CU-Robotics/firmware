#include <Arduino.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "sensors/ICM20649.hpp"
// Loop constants
#define LOOP_FREQ      1000
#define HEARTBEAT_FREQ 2

// Declare global objects
DR16 dr16;
rm_CAN can;

Timer loop_timer;
ICM20649 icm;
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
    can.init();
    SPI.begin();
    icm.init(icm.SPI);
    long long loopc = 0; // Loop counter for heartbeat

    
    float tempobs[9][3];
    float tempmotor[6];

<<<<<<< Updated upstream
    // Config config
    Serial.println("Configuring...");
    const Config* config = config_layer.configure(&comms);
    Serial.println("Configured!");

    //estimate micro and macro state
    estimator_manager.init(can_data, config);

    //generate controller outputs based on governed references and estimated state
    controller_manager.init(config);

    //set reference limits in the reference governor
    state.set_reference_limits(config->set_reference_limits);

    // variables for use in main
    float temp_state[STATE_LEN][3] = { 0 }; // Temp state array
    float temp_micro_state[NUM_MOTORS][MICRO_STATE_LEN] = { 0 }; // Temp micro state array
    float temp_reference[STATE_LEN][3] = { 0 }; //Temp governed state
    float target_state[STATE_LEN][3] = { 0 }; //Temp ungoverned state
    float hive_state_offset[STATE_LEN][3] = { 0 }; //Hive offset state
    float motor_inputs[NUM_MOTORS] = { 0 }; //Array for storing controller outputs to send to CAN

    // create a copy of the position and velocity kinematic matrixes since we'll be updating them
    float kinematics_pos[NUM_MOTORS][STATE_LEN] = { 0 }; //Position kinematics 
    memcpy(kinematics_pos, (*config).kinematics_p, sizeof((*config).kinematics_p));
    float kinematics_vel[NUM_MOTORS][STATE_LEN] = { 0 }; //Velocity kinematics
    memcpy(kinematics_vel, (*config).kinematics_v, sizeof((*config).kinematics_v));

    // used in the kinematics matrix
    float chassis_pos_to_motor_error = config->drive_conversion_factors[1];

    // manual controls variables
    float vtm_pos_x = 0;
    float vtm_pos_y = 0;
    float dr16_pos_x = 0;
    float dr16_pos_y = 0;
    float pos_offset_x = 0;
    float pos_offset_y = 0;

    // param to specify whether this is the first loop
    int count_one = 0;

    // whether we are in hive mode or not
    bool hive_toggle = false;

=======
    float ref[5][3] = {
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0}
    };
    /** 
    *   {s, s_dot, phi},
    *   {phi_dot, theta_ll, theta_ll_dot},
    *   {theta_lr, theta_lr_dot, theta_b},
    *   {theta_b_dot, s_ddot, phi_ddot},
    *   {psi_d, l_d, empty}
    */
>>>>>>> Stashed changes
    /*Testcontorl.init();
    Testobserver.init();
    can.read();
    imu.read();
    Testobserver.step(can.get_data(), imu.getdata(), tempobs);
    Testobserver.step(can.get_data(), imu.getdata(), tempobs);
    Testobserver.step(can.get_data(), imu.getdata(), tempobs);*/
    // Run 3 times for derivative values not get crazy (Maybe this works HAHA)
    // Main loop
    while (true) {
        // Read sensors
        dr16.read();
        can.write_motor_norm(CAN_3, 1, MG8016, 0.01);
        can.write_motor_norm(CAN_3, 2, MG8016, 0.01);
        icm.read();
        //Testobserver.step(can.get_data(), imu.getdata(), tempobs); // Calculate Observer values
        //Testcontorl.step(tempmotor, ref, tempobs); // Calculate motors motion
        Serial.println("ax");
        Serial.println(icm.get_accel_X());
        Serial.println("ay");
        Serial.println(icm.get_accel_Y());
        Serial.println("az");
        Serial.println(icm.get_accel_Z());
        Serial.println("gx");
        Serial.println(icm.get_gyro_X());
        Serial.println("gy");
        Serial.println(icm.get_gyro_Y());
        Serial.println("gz");
        Serial.println(icm.get_gyro_Z());
        
        // Write actuators
        /*if (!dr16.is_connected() || dr16.get_l_switch() == 1) {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.zero();
        } else if (dr16.is_connected() && dr16.get_l_switch() != 1) {
            // SAFETY OFF
            Serial.println("SAFTYOFF");
            can.write();
        }*/
        can.write();

        // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        loopc % (int)(1E3/float(HEARTBEAT_FREQ)) < (int)(1E3/float(5*HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6/(float)(LOOP_FREQ)));
    }
    return 0;
}


