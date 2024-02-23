#include <Arduino.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "controls/estimator_manager.hpp"
#include "controls/controller_manager.hpp"
#include "controls/state.hpp"

#include "sensors/RefSystem.hpp"


// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Declare global objects
DR16 dr16;
rm_CAN can;
RefSystem ref;

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
    ref.init();

    CANData *can_data = can.get_data();

    estimator_manager = new EstimatorManager(can_data);
    controller_manager = new ControllerManager();

    float gains[NUM_MOTORS][NUM_CONTROLLER_LEVELS][NUM_GAINS] = {0};
    int assigned_states[NUM_ESTIMATORS][STATE_LEN] = {0};
    int num_states_per_estimator[NUM_ESTIMATORS] = {3,3,1,1,16};
    float set_reference_limits[STATE_LEN][3][2] = {0};

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
    //Shooter Ball Speed
    set_reference_limits[5][0][0] = -UINT_MAX;
    set_reference_limits[5][0][1] = UINT_MAX;
    set_reference_limits[5][1][0] = 0;
    set_reference_limits[5][1][1] = 35;
    set_reference_limits[5][2][0] = -100000;
    set_reference_limits[5][2][1] = 100000;
    //Feeder Balls
    set_reference_limits[7][0][0] = -UINT_MAX;
    set_reference_limits[7][0][1] = UINT_MAX;
    set_reference_limits[7][1][0] = -40;
    set_reference_limits[7][1][1] = 40;
    set_reference_limits[7][2][0] = -100000;
    set_reference_limits[7][2][1] = 100000;
    // barrel switcher
    set_reference_limits[8][0][0] = 0; // i guessed on these make sure to change them
    set_reference_limits[8][0][1] = 40;
    set_reference_limits[8][1][0] = -1;
    set_reference_limits[8][1][1] = 1;
    set_reference_limits[8][2][0] = -1;
    set_reference_limits[8][2][1] = 1;

    state.set_reference_limits(set_reference_limits);

    float gain_1 = 0.001;

    gains[0][0][0] = 1; // Kp
    gains[0][0][1] = 0;   // Ki
    gains[0][0][2] = 0;   // Kd
    gains[0][1][0] = gain_1; // Kp
    gains[0][1][1] = 0;   // Ki
    gains[0][1][2] = 0;   // Kd
   
    gains[1][0][0] = 1; // Kp
    gains[1][0][1] = 0;   // Ki
    gains[1][0][2] = 0;   // Kd
    gains[1][1][0] = gain_1; // Kp
    gains[1][1][1] = 0;   // Ki
    gains[1][1][2] = 0;   // Kd

    gains[2][0][0] = 1; // Kp
    gains[2][0][1] = 0;   // Ki
    gains[2][0][2] = 0;   // Kd
    gains[2][1][0] = gain_1; // Kp
    gains[2][1][1] = 0;   // Ki
    gains[2][1][2] = 0;   // Kd
   
    gains[3][0][0] = 1; // Kp
    gains[3][0][1] = 0;   // Ki
    gains[3][0][2] = 0;   // Kd
    gains[3][1][0] = gain_1; // Kp
    gains[3][1][1] = 0;   // Ki
    gains[3][1][2] = 0;   // Kd

    gains[4][2][0] = 0; // Kp pos
    gains[4][2][1] = 0;   // Ki
    gains[4][2][2] = 0;   // Kd
    gains[4][2][3] = 0; // Kp vel
    gains[4][2][4] = 0;   // Ki
    gains[4][2][5] = 0;   // Kd

    gains[5][2][0] = 0; // Kp pos
    gains[5][2][1] = 0;   // Ki
    gains[5][2][2] = 0;   // Kd
    gains[5][2][3] = 0; // Kp vel
    gains[5][2][4] = 0;   // Ki
    gains[5][2][5] = 0;   // Kd

    gains[8][2][0] = 0; // Kp pos
    gains[8][2][1] = 0;   // Ki
    gains[8][2][2] = 0;   // Kd
    gains[8][2][3] = 0; // Kp vel
    gains[8][2][4] = 0;   // Ki
    gains[8][2][5] = 0;   // Kd

    gains[9][2][0] = 0; // Kp pos
    gains[9][2][1] = 0;   // Ki
    gains[9][2][2] = 0;   // Kd
    gains[9][2][3] = 0; // Kp vel
    gains[9][2][4] = 0;   // Ki
    gains[9][2][5] = 0;   // Kd

    gains[10][0][0] = 0; // Kp pos
    gains[10][0][1] = 0;   // Ki
    gains[10][0][2] = 0;   // Kd
    gains[10][1][0] = 0; // Kp pos
    gains[10][1][1] = 0;   // Ki
    gains[10][1][2] = 0;   // Kd
    
    gains[11][0][0] = 0; // Kp pos
    gains[11][0][1] = 0;   // Ki
    gains[11][0][2] = 0;   // Kd
    gains[11][1][0] = 0; // Kp pos
    gains[11][1][1] = 0;   // Ki
    gains[11][1][2] = 0;   // Kd

    gains[12][0][0] = 0; // Kp pos
    gains[12][0][1] = 0;   // Ki
    gains[12][0][2] = 0;   // Kd
    gains[12][1][0] = 0; // Kp pos
    gains[12][1][1] = 0;   // Ki
    gains[12][1][2] = 0;   // Kd

    assigned_states[0][0] = 0;
    assigned_states[0][1] = 1;
    assigned_states[0][2] = 2;

    assigned_states[1][0] = 2;
    assigned_states[1][1] = 3;
    assigned_states[1][2] = 4;

    assigned_states[2][0] = 5;

    assigned_states[3][0] = 6;
    
    for(int i = 0; i < NUM_MOTORS; i++) assigned_states[4][i] = i; 
    
    int controller_types[NUM_MOTORS][NUM_CONTROLLER_LEVELS]  = {{2,4,0},{2,4,0},{2,4,0},{2,4,0},{0,0,3},{0,0,3},{0,0,0},{0,0,0},{0,0,3},{0,0,3},{2,2,0},{2,2,0},{2,2,0},{0,0,0},{0,0,0},{0,0,0}};

    // intializes all controllers given the controller_types matrix
    for (int i = 0; i < NUM_CAN_BUSES; i++) {
        for (int j = 0; j < NUM_MOTORS_PER_BUS; j++) {
            for (int k = 0; k < NUM_CONTROLLER_LEVELS; k++) {
                    controller_manager->init_controller(i, j + 1, controller_types[(i*NUM_MOTORS_PER_BUS)+j][k], k, gains[(i*NUM_MOTORS_PER_BUS)+j][k]);
            }
        }
    }

    // initalize estimators
    estimator_manager->assign_states(assigned_states);

    for(int i = 0; i < NUM_ESTIMATORS; i++){
        estimator_manager->init_estimator(i+1, num_states_per_estimator[i]);
    }

    // imu calibration
    estimator_manager->calibrate_imus();

    long long loopc = 0;            // Loop counter for heartbeat
    float temp_state[STATE_LEN][3] = {0}; // Temp state array
    float temp_micro_state[NUM_MOTORS][MICRO_STATE_LEN] = {0}; // Temp micro state array
    float temp_reference[STATE_LEN][3] = {0};
    float target_state[STATE_LEN][3] = {0};
    float kinematics_pos[NUM_MOTORS][STATE_LEN] = {0};
    float kinematics_vel[NUM_MOTORS][STATE_LEN] = {0};
    float motor_inputs[NUM_MOTORS] = {0};
    int governor_type[STATE_LEN] = {2, 2, 2, 1, 1, 2, 2, 2};

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

    target_state[2][1] = 3;
    // target_state[0][0] = 3;
    
    // Main loop
    while (true)
    {
        can.read();
        dr16.read();
        ref.read();

        // Read sensors
        estimator_manager->read_sensors();
        estimator_manager->step(temp_state, temp_micro_state);
        
        state.set_estimate(temp_state);
        state.step_reference(target_state, governor_type);
        state.get_reference(temp_reference);
        
        // Controls code goes here
        controller_manager->step(temp_reference, temp_state, temp_micro_state, kinematics_pos, kinematics_vel, motor_inputs);
        
        for (int j = 0; j < 2; j++)
        {
            for (int i = 0; i < NUM_MOTORS_PER_BUS; i++)
            {
                can.write_motor_norm(j, i+1, C620, motor_inputs[(j*NUM_MOTORS_PER_BUS)+i]);
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
        { // prints the full micro estimate
            Serial.printf("[");
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                Serial.printf("[");
                for(int j = 0; j < MICRO_STATE_LEN; j++) {
                    Serial.print(temp_micro_state[i][j]);
                    if (j != MICRO_STATE_LEN - 1)
                    Serial.printf(", ");
                }
                Serial.printf("]");
                if (i != NUM_MOTORS - 1)
                    Serial.printf(", ");
            }
            Serial.printf("] \n");
        }

        if (true)
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
