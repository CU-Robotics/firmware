#include <Arduino.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "controls/estimator_manager.hpp"
#include "controls/controller_manager.hpp"
#include "controls/state.hpp"
#include "comms/usb_hid.hpp"
#include "comms/config_layer.hpp"
#include "sensors/RefSystem.hpp"
#include "sensors/d200.hpp"
#include "sensors/ACS712.hpp"
#include <FastLED.h>

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Declare global objects
DR16 dr16;
rm_CAN can;
RefSystem ref;
HIDLayer comms;

CFastLED led_perhaps;

D200LD14P lidar1(&Serial4, 0);
D200LD14P lidar2(&Serial5, 1);

ACS712 current_sensor;

ConfigLayer config_layer;
Config config;

Timer loop_timer;
Timer stall_timer;
Timer control_input_timer;

EstimatorManager* estimator_manager;
ControllerManager* controller_manager;
State state;

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
    long long loopc = 0; // Loop counter for heartbeat

    Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    print_logo();

    // Execute setup functions
    pinMode(13, OUTPUT);

    //initialize objects
    can.init();
    dr16.init();
    ref.init();
    comms.init();

    //can data pointer so we don't pass around rm_CAN object
    CANData* can_data = can.get_data();

    uint8_t packet_subsection_sizes[MAX_CONFIG_PACKETS] = { 0 };
    CommsPacket config_packets[MAX_CONFIG_PACKETS];
    // Config config
    
    Serial.println("Configuring...");
    while(!config_layer.is_configured()) {
        comms.ping();
        config_layer.process(comms.get_incoming_packet(), comms.get_outgoing_packet());
    }
    config_layer.get_config_packets(config_packets, packet_subsection_sizes);
    config.fill_data(config_packets, packet_subsection_sizes);
    Serial.println("Configured!");

    //estimate micro and macro state
    estimator_manager = new EstimatorManager(can_data, config);
    //generate controller outputs based on governed references and estimated state
    controller_manager = new ControllerManager();

    //gains for each motor and controller
    float gains[NUM_MOTORS][NUM_CONTROLLER_LEVELS][NUM_GAINS] = { 0 };
    memcpy(gains, config.gains, sizeof(config.gains));
    //which states each estimator estimates
    float assigned_states[NUM_ESTIMATORS][STATE_LEN] = { 0 };
    memcpy(assigned_states, config.assigned_states, sizeof(config.assigned_states));
    //number of states each estimator estimates
    float num_states_per_estimator[NUM_ESTIMATORS] = { 0 };
    memcpy(num_states_per_estimator, config.num_states_per_estimator, sizeof(config.num_states_per_estimator));
    //reference limits of our reference governor. Used to turn an ungoverned reference into a governed reference to send to ControllerManager
    float set_reference_limits[STATE_LEN][3][2] = { 0 };
    memcpy(set_reference_limits, config.set_reference_limits, sizeof(config.set_reference_limits));

    float governor_types[STATE_LEN] = { 0 }; //Position vs Velcity governor
    memcpy(governor_types, config.governor_types, sizeof(config.governor_types));

    float kinematics_pos[NUM_MOTORS][STATE_LEN] = { 0 }; //Position kinematics 
    memcpy(kinematics_pos, config.kinematics_p, sizeof(config.kinematics_p));
    float kinematics_vel[NUM_MOTORS][STATE_LEN] = { 0 }; //Velocity kinematics
    memcpy(kinematics_vel, config.kinematics_v, sizeof(config.kinematics_v));
    
    float controller_types[NUM_MOTORS][NUM_CONTROLLER_LEVELS] = { 0 };
    memcpy(controller_types, config.controller_types, sizeof(config.controller_types));
    
    float chassis_pos_to_motor_error = config.drive_conversion_factors[1];

    //set reference limits in the reference governor
    state.set_reference_limits(set_reference_limits);

    // intializes all controllers given the controller_types matrix
    for (int i = 0; i < NUM_CAN_BUSES; i++)
    {
        for (int j = 0; j < NUM_MOTORS_PER_BUS; j++)
        {
            for (int k = 0; k < NUM_CONTROLLER_LEVELS; k++)
            {
                controller_manager->init_controller(i, j + 1, controller_types[(i * NUM_MOTORS_PER_BUS) + j][k], k, gains[(i * NUM_MOTORS_PER_BUS) + j][k]);
            }
        }
    }

    // initalize estimators
    estimator_manager->assign_states(assigned_states);
   
    for(int i = 0; i < NUM_ESTIMATORS; i++){
        Serial.printf("Init Estimator %d\n", config.estimators[i]);

        if(config.estimators[i] != 0){
            estimator_manager->init_estimator(config.estimators[i], (int) num_states_per_estimator[i]);
        }
    }

    // imu calibration
    estimator_manager->calibrate_imus();

    // variables for use in main
    float temp_state[STATE_LEN][3] = { 0 }; // Temp state array
    float temp_micro_state[NUM_MOTORS][MICRO_STATE_LEN] = { 0 }; // Temp micro state array
    float temp_reference[STATE_LEN][3] = { 0 }; //Temp governed state
    float target_state[STATE_LEN][3] = { 0 }; //Temp ungoverned state
    float hive_state_offset[STATE_LEN][3] = { 0 }; //Hive offset state
    float motor_inputs[NUM_MOTORS] = { 0 }; //Array for storing controller outputs to send to CAN

    float dr16_pos_x = 0;
    float dr16_pos_y = 0;
    float pos_offset_x = 0;
    float pos_offset_y = 0;
    int vtm_pos_x = 0;
    int vtm_pos_y = 0;
    int count_one = 0;
    float chassis_velocity_x = 0;
    float chassis_velocity_y = 0;
    float chassis_pos_x = 0;
    float chassis_pos_y = 0;

    bool hive_toggle = false;

    // Main loop
    while (true) {
        //read everything
        can.read();
        dr16.read();
        ref.read();
        current_sensor.read();
        lidar1.read();
        lidar2.read();
        
        //handle read/write
        comms.ping();

        CommsPacket* incoming = comms.get_incoming_packet();
        CommsPacket* outgoing = comms.get_outgoing_packet();

        //manual controls on firmware
        float delta = control_input_timer.delta();
        dr16_pos_x += dr16.get_mouse_x() * 0.05 * delta;
        dr16_pos_y += dr16.get_mouse_y() * 0.05 * delta;

        vtm_pos_x += ref.ref_data.kbm_interaction.mouse_speed_x * 0.05 * delta;
        vtm_pos_y += ref.ref_data.kbm_interaction.mouse_speed_y * 0.05 * delta;
        if(config.governor_types[0] == 2){
            chassis_velocity_x = -dr16.get_l_stick_y() * 5.4
                                    + (-ref.ref_data.kbm_interaction.key_w + ref.ref_data.kbm_interaction.key_s) * 2.5
                                    + (-dr16.keys.w + dr16.keys.s) * 2.5;
            chassis_velocity_y = dr16.get_l_stick_x() * 5.4
                                    + (ref.ref_data.kbm_interaction.key_d - ref.ref_data.kbm_interaction.key_a) * 2.5
                                    + (dr16.keys.d - dr16.keys.a) * 2.5;
        } else if (config.governor_types[0] == 1){
            chassis_pos_x = dr16.get_l_stick_x() * 2 + pos_offset_x;
            chassis_pos_y = dr16.get_l_stick_y() * 2 + pos_offset_y;
        }
        float chassis_spin = dr16.get_wheel() * 25;

        float pitch_target = 1.57
                            + -dr16.get_r_stick_y() * 0.3
                            + dr16_pos_y
                            + vtm_pos_y;
        float yaw_target = -dr16.get_r_stick_x() * 1.5
                            - dr16_pos_x
                            - vtm_pos_x;
        float fly_wheel_target = (dr16.get_r_switch() == 1 || dr16.get_r_switch() == 3) ? 18 : 0; //m/s
        float feeder_target = (((dr16.get_l_mouse_button() || ref.ref_data.kbm_interaction.button_left) && dr16.get_r_switch() != 2) || dr16.get_r_switch() == 1) ? 10 : 0;

        target_state[0][0] = chassis_pos_x;
        target_state[0][1] = chassis_velocity_x;
        target_state[1][0] = chassis_pos_y;
        target_state[1][1] = chassis_velocity_y;
        target_state[2][1] = chassis_spin;
        target_state[3][0] = yaw_target;
        target_state[3][1] = 0;
        target_state[4][0] = pitch_target;
        target_state[4][1] = 0;

        target_state[5][1] = fly_wheel_target;
        target_state[6][1] = feeder_target;
        // target_state[7][0] = dr16.get_r_switch() == 2 ? 1 : -1;
        target_state[7][0] = 1;

        // if the left switch is all the way down use Hive controls
        if(dr16.get_l_switch() == 2) {
            incoming->get_target_state(target_state);
            // if you just switched to hive controls, set the reference to the current state
            if(hive_toggle){
                state.set_reference(temp_state);
                hive_toggle = false;
            } 
        }
        // when in teensy control mode reset hive toggle
        if(dr16.get_l_switch() == 3) {
            if(!hive_toggle){
                pos_offset_x = temp_state[0][0];
                pos_offset_y = temp_state[1][0];
            }
            hive_toggle = true;
        }
        
        // Read sensors
        estimator_manager->read_sensors();

        //step estimates and construct estimated state
        // Serial.printf("step\n");
        
        if(incoming->get_hive_override_request() == 1) {
            incoming->get_hive_override_state(hive_state_offset);
            for(int i = 0; i < STATE_LEN; i++) {
                for(int j = 0; j < 3; j++) {
                    temp_state[i][j] = hive_state_offset[i][j];
                    // Serial.printf("override: %d, %d\n", i, j);
                }
            }
        }

        estimator_manager->step(temp_state, temp_micro_state, incoming->get_hive_override_request());
        // Serial.printf("estimated\n");
        //if first loop set target state to estimated state
        if (count_one == 0) {
            temp_state[7][0] = 0;
            state.set_reference(temp_state);
            count_one++;
        }

        //reference govern
        state.set_estimate(temp_state);
        state.step_reference(target_state, governor_types);
        state.get_reference(temp_reference);

        // Update the kinematics of x,y states, as the kinematics change when chassis angle changes
        kinematics_vel[0][0] = -sin(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_vel[0][1] = cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        // motor 2 back right
        kinematics_vel[1][0] = cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_vel[1][1] = sin(temp_state[2][0]) * chassis_pos_to_motor_error;
        // motor 3 back left
        kinematics_vel[2][0] = sin(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_vel[2][1] = -cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        // motor 4 front left
        kinematics_vel[3][0] = -cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_vel[3][1] = -sin(temp_state[2][0]) * chassis_pos_to_motor_error;

        // Update the kinematics of x,y states, as the kinematics change when chassis angle changes
        kinematics_pos[0][0] = -sin(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_pos[0][1] = cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        // motor 2 back right
        kinematics_pos[1][0] = cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_pos[1][1] = sin(temp_state[2][0]) * chassis_pos_to_motor_error;
        // motor 3 back left
        kinematics_pos[2][0] = sin(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_pos[2][1] = -cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        // motor 4 front left
        kinematics_pos[3][0] = -cos(temp_state[2][0]) * chassis_pos_to_motor_error;
        kinematics_pos[3][1] = -sin(temp_state[2][0]) * chassis_pos_to_motor_error;
        //generate motor outputs from controls
        controller_manager->step(temp_reference, temp_state, temp_micro_state, kinematics_pos, kinematics_vel, motor_inputs);

        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < NUM_MOTORS_PER_BUS; i++) {
                can.write_motor_norm(j, i + 1, C620, motor_inputs[(j * NUM_MOTORS_PER_BUS) + i]);
                if (j == 1 && i == 4)
                    can.write_motor_norm(j, i + 1, C610, motor_inputs[(j * NUM_MOTORS_PER_BUS) + i]);
            }
        }

        // Serial.printf("Current: %f\n", val);
        // construct sensor data packet
        SensorData sensor_data;
        // set dr16 raw data
        memcpy(sensor_data.raw + SENSOR_DR16_OFFSET, dr16.get_raw(), DR16_PACKET_SIZE);
        // set lidars
        uint8_t lidar_data[D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE] = { 0 };
        lidar1.export_data(lidar_data);
        
        memcpy(sensor_data.raw + SENSOR_LIDAR1_OFFSET, lidar_data, D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE);
        lidar2.export_data(lidar_data);
        memcpy(sensor_data.raw + SENSOR_LIDAR2_OFFSET, lidar_data, D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE);

        
        // construct ref data packet
        uint8_t ref_data_raw[180] = { 0 };
        ref.get_data_for_comms(ref_data_raw);

        // set the outgoing packet
        outgoing->set_id((uint16_t)loopc);
        outgoing->set_info(0x0000);
        outgoing->set_time(millis() / 1000.0);
        outgoing->set_sensor_data(&sensor_data);
        outgoing->set_ref_data(ref_data_raw);
        outgoing->set_estimated_state(temp_state);
        //  SAFETY MODE
        if (dr16.is_connected() && (dr16.get_l_switch() == 2 || dr16.get_l_switch() == 3) && config_layer.is_configured()) {
            // SAFETY OFF
            can.write();
        } else {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.zero();
        }

        // Serial.print("estimate:");
        //     for (int i = 2; i < 4; i++) {
        //         Serial.printf("[");
        //         for (int j = 0; j < 3; j++) {
        //             Serial.printf("%f ,", temp_state[i][j]);
        //         }
        //         Serial.print("] ");
        //     }
        //     Serial.print("\nreference: ");
        //     for (int i = 2; i < 4; i++) {
        //         Serial.printf("[");
        //         for (int j = 0; j < 3; j++) {
        //             Serial.printf("%f ,", temp_reference[i][j]);
        //         }
        //         Serial.print("] ");
        //     }
        //     Serial.print("\nmotor inputs: ");
        //     for (int i = 4; i < 6; i++) {
        //             Serial.printf("%f ,", motor_inputs[i]);
        //     }
        //     Serial.println();

        // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
        float dt = stall_timer.delta();
        if (dt > 0.002) Serial.printf("Slow loop with dt: %f\n", dt);
    }
    return 0;
}


