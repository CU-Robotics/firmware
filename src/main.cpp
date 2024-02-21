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

    // controller_manager = Control::get_instance();

    estimator_manager = new EstimatorManager(can_data);
    controller_manager = new ControllerManager();

    float gains_null[NUM_GAINS];
    float gains_1[NUM_GAINS];
    int assigned_states[NUM_ESTIMATORS][STATE_LEN];
    float set_reference_limits[STATE_LEN][3][2];

    set_reference_limits[2][1][0] = -105.626;
    set_reference_limits[2][1][1] = 105.626;


    set_reference_limits[2][2][0] = -1;
    set_reference_limits[2][2][1] = 1;

    state.set_reference_limits(set_reference_limits);

    gains_1[0] = 0.003; // Kp
    gains_1[1] = 0;   // Ki
    gains_1[2] = 0;   // Kd

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

    controller_manager->init_controller(CAN_1, 1, 2, gains_1);
    controller_manager->init_controller(CAN_1, 2, 2, gains_1);
    controller_manager->init_controller(CAN_1, 3, 2, gains_1);
    controller_manager->init_controller(CAN_1, 4, 2, gains_1);

    estimator_manager->assign_states(assigned_states);

    estimator_manager->init_estimator(1, 3);
    estimator_manager->init_estimator(2, 3);
    estimator_manager->calibrate_imus();

    long long loopc = 0;            // Loop counter for heartbeat
    float temp_state[STATE_LEN][3]; // Temp state array
    float temp_reference[STATE_LEN][3];
    float target_state[STATE_LEN][3];
    float kinematics[NUM_MOTORS][STATE_LEN];
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
            kinematics[i][j] = 0;
        }
    }
    float chassis_angle_to_motor_error = ((.200*9.17647058824)/.0516);
    kinematics[0][2] = chassis_angle_to_motor_error;    
    kinematics[1][2] = chassis_angle_to_motor_error;
    kinematics[2][2] = chassis_angle_to_motor_error;
    kinematics[3][2] = chassis_angle_to_motor_error;


    target_state[2][1] = 6;
    

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
        controller_manager->step(temp_reference, temp_state, kinematics, motor_inputs);

        for (int j = 0; j < 2; j++)
        {
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                can.write_motor_norm(j, i+1, C620, motor_inputs[(j*NUM_MOTORS)+i]);
            }
        }

        Serial.println(motor_inputs[0]);

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

        // Write actuators
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
