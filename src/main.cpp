#include <Arduino.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "controls/estimator_manager.hpp"
#include "controls/controller_manager.hpp"
#include "controls/state.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Declare global objects
DR16 dr16;
rm_CAN can;
Timer loop_timer;
EstimatorManager *estimator_manager;
ControllerManager *controller_manager;
State state;

// DONT put anything else in this function. It is not a setup function
void print_logo()
{
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
        Serial.println("\n\033[1;92mFW Ver. 2.1.0");
        Serial.printf("\nLast Built: %s at %s", __DATE__, __TIME__);
        Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}

// Master loop
int main()
{
    Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    print_logo();

    // Execute setup functions
    pinMode(13, OUTPUT);

    can.init();
    dr16.init();

    CANData *can_data = can.get_data();

    estimator_manager = new EstimatorManager(can_data);
    controller_manager = new ControllerManager();

    float gains_null[NUM_GAINS];
    float gains_chassis[NUM_GAINS];
    float gains_pitch[NUM_GAINS];
    float gains_yaw[NUM_GAINS];
    float gains_flywheel[NUM_GAINS];
    float gains_feeder[NUM_GAINS];
    int assigned_states[NUM_ESTIMATORS][STATE_LEN];
    float set_reference_limits[STATE_LEN][3][2];

    //x pos
    set_reference_limits[0][0][0] = -UINT_MAX;
    set_reference_limits[0][0][1] = UINT_MAX;
    set_reference_limits[0][1][0] = -5.37952195918;
    set_reference_limits[0][1][1] = 5.37952195918;
    set_reference_limits[0][2][0] = -1;
    set_reference_limits[0][2][1] = 1;
    //y pos
    set_reference_limits[1][0][0] = -UINT_MAX;
    set_reference_limits[1][0][1] = UINT_MAX;
    set_reference_limits[1][1][0] = -5.37952195918;
    set_reference_limits[1][1][1] = 5.37952195918;
    set_reference_limits[1][2][0] = -1;
    set_reference_limits[1][2][1] = 1;
    //chassis angle (psi)
    set_reference_limits[2][0][0] = -UINT_MAX;
    set_reference_limits[2][0][1] = UINT_MAX;
    set_reference_limits[2][1][0] = -29.3917681162;
    set_reference_limits[2][1][1] = 29.3917681162;
    set_reference_limits[2][2][0] = -1;
    set_reference_limits[2][2][1] = 1;
    //yaw
    set_reference_limits[3][0][0] = -UINT_MAX;
    set_reference_limits[3][0][1] = UINT_MAX;
    set_reference_limits[3][1][0] = -49.4827627617;
    set_reference_limits[3][1][1] = 49.4827627617;
    set_reference_limits[3][2][0] = -1;
    set_reference_limits[3][2][1] = 1;
    //pitch
    set_reference_limits[4][0][0] = 1.92;
    set_reference_limits[4][0][1] = 0.9;
    set_reference_limits[4][1][0] = -70.1181276577;
    set_reference_limits[4][1][1] = 70.1181276577;
    set_reference_limits[4][2][0] = -1;
    set_reference_limits[4][2][1] = 1;
    //flywheel Left
    set_reference_limits[5][0][0] = -UINT_MAX;
    set_reference_limits[5][0][1] = UINT_MAX;
    set_reference_limits[5][1][0] = -969.28;
    set_reference_limits[5][1][1] = 969.28;
    set_reference_limits[5][2][0] = -1000;
    set_reference_limits[5][2][1] = 1000;
    //flywheel Right
    set_reference_limits[6][0][0] = -UINT_MAX;
    set_reference_limits[6][0][1] = UINT_MAX;
    set_reference_limits[6][1][0] = -969.28;
    set_reference_limits[6][1][1] = 969.28;
    set_reference_limits[6][2][0] = -1000;
    set_reference_limits[6][2][1] = 1000;
    //feeder
    set_reference_limits[7][0][0] = -UINT_MAX;
    set_reference_limits[7][0][1] = UINT_MAX;
    set_reference_limits[7][1][0] = -10;
    set_reference_limits[7][1][1] = 10;
    set_reference_limits[7][2][0] = -1;
    set_reference_limits[7][2][1] = 1;
    // barrel switcher
    set_reference_limits[8][0][0] = 0; // i guessed on these make sure to change them
    set_reference_limits[8][0][1] = 40;
    set_reference_limits[8][1][0] = -1;
    set_reference_limits[8][1][1] = 1;
    set_reference_limits[8][2][0] = -1;
    set_reference_limits[8][2][1] = 1;

    state.set_reference_limits(set_reference_limits);

    gains_chassis[0] = 0.003; // Kp
    gains_chassis[1] = 0;   // Ki
    gains_chassis[2] = 0;   // Kd

    gains_pitch[0] = 0; // Kp pos
    gains_pitch[1] = 0;   // Ki
    gains_pitch[2] = 0;   // Kd
    gains_pitch[3] = 0; // Kp vel
    gains_pitch[4] = 0;   // Ki
    gains_pitch[5] = 0;   // Kd

    gains_yaw[0] = 0; // Kp pos
    gains_yaw[1] = 0;   // Ki
    gains_yaw[2] = 0;   // Kd
    gains_yaw[3] = 0; // Kp vel
    gains_yaw[4] = 0;   // Ki
    gains_yaw[5] = 0;   // Kd

    gains_feeder[0] = 0; // Kp
    gains_feeder[1] = 0;   // Ki
    gains_feeder[2] = 0;   // Kd

    gains_flywheel[0] = 0; // Kp
    gains_flywheel[1] = 0;   // Ki
    gains_flywheel[2] = 0;   // Kd

    for (int i = 0; i < NUM_ESTIMATORS; i++)
    {
        for (int j = 0; j < STATE_LEN; j++)
        {
            assigned_states[i][j] = 0;
        }
    }

    assigned_states[0][0] = 0;
    assigned_states[0][1] = 1;
    assigned_states[0][2] = 2;

    assigned_states[1][0] = 2;
    assigned_states[1][1] = 3;
    assigned_states[1][2] = 4;

    for (int i = 0; i < NUM_CAN_BUSES; i++)
    {
        for (int j = 0; j < NUM_MOTORS_PER_BUS; j++)
        {
            controller_manager->init_controller(i, j + 1, 0, gains_null);
        }
    }
    // can_1
    // drive controllers (id 1,2,3,4) (Velocity PID Control)
    controller_manager->init_controller(CAN_1, 1, 1, gains_chassis);
    controller_manager->init_controller(CAN_1, 2, 1, gains_chassis);
    controller_manager->init_controller(CAN_1, 3, 1, gains_chassis);
    controller_manager->init_controller(CAN_1, 4, 1, gains_chassis);
    // yaw controllers (id 4,5) (Full State Control)
    controller_manager->init_controller(CAN_1, 5, 3, gains_yaw);
    controller_manager->init_controller(CAN_1, 6, 3, gains_yaw);
    // can_2
    // pitch controllers (id 1,2) (Full State Control)
    controller_manager->init_controller(CAN_2, 1, 3, gains_pitch);
    controller_manager->init_controller(CAN_2, 2, 3, gains_pitch);
    // flywheel controllers (id 3,4) (Velocity PID Control)
    controller_manager->init_controller(CAN_2, 3, 2, gains_flywheel);
    controller_manager->init_controller(CAN_2, 4, 2, gains_flywheel);
    // feeder controller (id 5) (Velocity PID Control)
    controller_manager->init_controller(CAN_2, 5, 2, gains_feeder);
    // barrel switcher (id 6) (Pos PID Control)
    //controller_manager->init_controller(CAN_2, 6, 1, gains_switcher);

    // initalize estimators
    estimator_manager->assign_states(assigned_states);
    estimator_manager->init_estimator(1, 3);
    estimator_manager->init_estimator(2, 3);
    // imu calibration
    estimator_manager->calibrate_imus();

    long long loopc = 0;            // Loop counter for heartbeat
    float temp_state[STATE_LEN][3]; // Temp state array
    float temp_reference[STATE_LEN][3];
    float target_state[STATE_LEN][3];
    float kinematics_pos[NUM_MOTORS][STATE_LEN];
    float kinematics_vel[NUM_MOTORS][STATE_LEN];
    float motor_inputs[NUM_MOTORS];
    int controller_type[STATE_LEN] = {2, 2, 2, 1, 1, 2, 2, 2};

    for (int i = 0; i < STATE_LEN; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            target_state[i][j] = 0;
        }
    }

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        for (int j = 0; j < STATE_LEN; j++)
        {
            kinematics_pos[i][j] = 0;
            kinematics_vel[i][j] = 0;
        }
    }
    float chassis_angle_to_motor_error = ((.1835*9.17647058824)/.0516);
    float chassis_pos_to_motor_error = ((9.17647058824)/.0516);
    // motor 1 front right Can_1
    kinematics_vel[0][0] = chassis_angle_to_motor_error;  
    kinematics_vel[0][1] = chassis_angle_to_motor_error;  
    kinematics_vel[0][2] = chassis_angle_to_motor_error; 
    // motor 2 back right
    kinematics_vel[1][0] = chassis_angle_to_motor_error;
    kinematics_vel[1][1] = chassis_angle_to_motor_error;
    kinematics_vel[1][2] = chassis_angle_to_motor_error;
    // motor 3 back left
    kinematics_vel[2][0] = chassis_angle_to_motor_error;
    kinematics_vel[2][1] = chassis_angle_to_motor_error;
    kinematics_vel[2][2] = chassis_angle_to_motor_error;
    // motor 4 front left
    kinematics_vel[3][0] = chassis_angle_to_motor_error;
    kinematics_vel[3][1] = chassis_angle_to_motor_error;
    kinematics_vel[3][2] = chassis_angle_to_motor_error;
    // motor 5 yaw 1
    kinematics_vel[0][0] = chassis_angle_to_motor_error;  
    kinematics_vel[0][1] = chassis_angle_to_motor_error;  
    // motor 6 yaw 2
    kinematics_vel[1][1] = chassis_angle_to_motor_error;
    kinematics_vel[1][2] = chassis_angle_to_motor_error;

    // motor 1 pitch 1 Can_2
    kinematics_vel[2][0] = chassis_angle_to_motor_error;
    kinematics_vel[2][1] = chassis_angle_to_motor_error;
    kinematics_vel[2][2] = chassis_angle_to_motor_error;
    // motor 2 pitch 2
    kinematics_vel[3][0] = chassis_angle_to_motor_error;
    kinematics_vel[3][1] = chassis_angle_to_motor_error;
    kinematics_vel[3][2] = chassis_angle_to_motor_error;
    // motor 3 flywheel 1 
    kinematics_vel[2][0] = chassis_angle_to_motor_error;
    kinematics_vel[2][1] = chassis_angle_to_motor_error;
    kinematics_vel[2][2] = chassis_angle_to_motor_error;
    // motor 2 flywheel 2 
    kinematics_vel[3][0] = chassis_angle_to_motor_error;
    kinematics_vel[3][1] = chassis_angle_to_motor_error;
    kinematics_vel[3][2] = chassis_angle_to_motor_error;
    // motor 1 feeder
    kinematics_vel[2][0] = chassis_angle_to_motor_error;
    kinematics_vel[2][1] = chassis_angle_to_motor_error;
    kinematics_vel[2][2] = chassis_angle_to_motor_error;
    // motor 2 switcher
    kinematics_vel[3][0] = chassis_angle_to_motor_error;
    kinematics_vel[3][1] = chassis_angle_to_motor_error;
    kinematics_vel[3][2] = chassis_angle_to_motor_error;


    // target_state[2][1] = 6;
    target_state[0][0] = 3;
    

    // Main loop
    while (true)
    {

        can.read();
        dr16.read();

        // Read sensors
        estimator_manager->read_sensors();
        estimator_manager->step(temp_state);

        state.set_estimate(temp_state);
        state.step_reference(target_state, controller_type);
        state.get_reference(temp_reference);

        // Controls code goes here
        controller_manager->step(temp_reference, temp_state, kinematics_vel, motor_inputs);

        for (int j = 0; j < 2; j++)
        {
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                // can.write_motor_norm(j, i+1, C620, motor_inputs[(j*NUM_MOTORS)+i]);
            }
        }

        if (true)
        { // prints the full motor input vector
            Serial.printf("[");
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                Serial.print(motor_inputs[i]);
                if (i != NUM_MOTORS - 1)
                    Serial.printf(", ");
            }
            Serial.printf("] \n");
        }

        if (false)
        { // prints the estimated state
            for (int i = 0; i < STATE_LEN-27; i++) {
            Serial.printf("[");
            for (int j = 0; j < 3; j++)
            {
                Serial.printf("%.3f",temp_state[i][j]);
                if (j != 3 - 1)
                    Serial.printf(", ");
            }
            Serial.printf("]");
            }
            Serial.println();
        }

        // Write actuators (Safety Code)
        if (!dr16.is_connected() || dr16.get_l_switch() == 1)
        {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.zero();
        }
        else if (dr16.is_connected() && dr16.get_l_switch() != 1)
        {
            // SAFETY OFF
            can.write();
        }

        // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
    }

    delete estimator_manager;
    return 0;
}
