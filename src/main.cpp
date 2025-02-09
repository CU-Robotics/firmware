#include <Arduino.h>

#include "git_info.h"

#include "utils/profiler.hpp"
#include "sensors/d200.hpp"
#include "sensors/Transmitter.hpp"
#include "sensors/ET16S.hpp"
#include "sensors/StereoCamTrigger.hpp"
#include "controls/estimator_manager.hpp"
#include "controls/controller_manager.hpp"

#include <TeensyDebug.h>
#include "sensors/LEDBoard.hpp"
#include "data_packet.hpp"
#include "sensor_constants.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Declare global objects
rm_CAN can;
RefSystem ref;
HIDLayer comms;
ACS712 current_sensor;

D200LD14P lidar1(&Serial4, 0);
D200LD14P lidar2(&Serial5, 1);

StereoCamTrigger stereoCamTrigger(60);

ConfigLayer config_layer;

Profiler prof;

Timer loop_timer;
Timer stall_timer;
Timer control_input_timer;

EstimatorManager estimator_manager;
ControllerManager controller_manager;

Governor governor;

LEDBoard led;

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
        Serial.printf("\nGit Hash: %s", GIT_COMMIT_HASH);
        Serial.printf("\nGit Branch: %s", GIT_BRANCH);
        Serial.printf("\nCommit Message: %s", GIT_COMMIT_MSG);
        Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}

// Master loop
int main() {
    uint32_t loopc = 0; // Loop counter for heartbeat

    Serial.begin(112500); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);

    print_logo();

    // Execute setup functions
    pinMode(LED_BUILTIN, OUTPUT);

    led.init();
	Transmitter transmitter;
	TransmitterType t_type = transmitter.who_am_i();
	if (t_type == TransmitterType::DR16){
		DR16 dr16;
		transmitter = dr16;
	}
	else if (t_type == TransmitterType::ET16S){
		ET16S wfly;
		transmitter = wfly;
	}
	Serial.println();
	Serial.print("size of DR16:");
	Serial.print(sizeof(DR16));
	Serial.println();
	Serial.print("size of ET16S:");
	Serial.print(sizeof(ET16S));
	Serial.println();
	Serial.print("size of Transmitter:");
	Serial.print(sizeof(Transmitter));
	Serial.println();
    can.init();
    transmitter.init();
    ref.init();
    comms.init();



	
    //can data pointer so we don't pass around rm_CAN object
    // TODO: extern the can_data object
    CANData* can_data = can.get_data();

	
    // Config config
    Serial.println("Configuring...");
	while(true){}
    const Config* config = config_layer.configure(&comms);
    Serial.println("Configured!");
    //estimate micro and macro state
    estimator_manager.init(can_data, config);

    //generate controller outputs based on governed references and estimated state
    controller_manager.init(&can, config);

    //set reference limits in the reference governor
    governor.set_reference_limits(config->set_reference_limits);

    // variables for use in main
    float temp_state[STATE_LEN][3] = { 0 }; // Temp state array
    float temp_micro_state[NUM_MOTORS][MICRO_STATE_LEN] = { 0 }; // Temp micro state array
    float temp_reference[STATE_LEN][3] = { 0 }; //Temp governed state
    float target_state[STATE_LEN][3] = { 0 }; //Temp ungoverned state
    float hive_state_offset[STATE_LEN][3] = { 0 }; //Hive offset state
    // float motor_inputs[NUM_MOTORS] = { 0 }; //Array for storing controller outputs to send to CAN

    // manual controls variables
    float vtm_pos_x = 0;
    float vtm_pos_y = 0;
    float transmitter_pos_x = 0;
    float transmitter_pos_y = 0;
    float pos_offset_x = 0;
    float pos_offset_y = 0;

    // param to specify whether this is the first loop
    int count_one = 0;

    // whether we are in hive mode or not
    bool hive_toggle = false;

    Serial.println("Entering main loop...\n");

    // Main loop
    while (true) {
        // read main sensors
        can.read();
        transmitter.read();
        ref.read();
        lidar1.read();
        lidar2.read();

        // read and write comms packets
        comms.ping();
        CommsPacket* incoming = comms.get_incoming_packet();
        CommsPacket* outgoing = comms.get_outgoing_packet();

        // check whether this packet is a config packet
        if (incoming->raw[3] == 1) {
            Serial.println("\n\nConfig request received, reconfiguring from comms!\n\n");
            // trigger safety mode
            can.zero();
            config_layer.reconfigure(&comms);
        }

        // print loopc every second to verify it is still alive
        if (loopc % 1000 == 0) {
            Serial.println(loopc);
        }

        // manual controls on firmware

        float delta = control_input_timer.delta();
        transmitter_pos_x += transmitter.get_mouse_x() * 0.05 * delta;
        transmitter_pos_y += transmitter.get_mouse_y() * 0.05 * delta;

        vtm_pos_x += ref.ref_data.kbm_interaction.mouse_speed_x * 0.05 * delta;
        vtm_pos_y += ref.ref_data.kbm_interaction.mouse_speed_y * 0.05 * delta;
        
        float chassis_vel_x = 0;
        float chassis_vel_y = 0;
        float chassis_pos_x = 0;
        float chassis_pos_y = 0;
        if (config->governor_types[0] == 2) {   // if we should be controlling velocity
            chassis_vel_x = transmitter.get_l_stick_y() * 5.4
                + (-ref.ref_data.kbm_interaction.key_w + ref.ref_data.kbm_interaction.key_s) * 2.5
                + (-transmitter.keys.w + transmitter.keys.s) * 2.5;
            chassis_vel_y = -transmitter.get_l_stick_x() * 5.4
                + (ref.ref_data.kbm_interaction.key_d - ref.ref_data.kbm_interaction.key_a) * 2.5
                + (transmitter.keys.d - transmitter.keys.a) * 2.5;
        } else if (config->governor_types[0] == 1) { // if we should be controlling position
            chassis_pos_x = transmitter.get_l_stick_x() * 2 + pos_offset_x;
            chassis_pos_y = transmitter.get_l_stick_y() * 2 + pos_offset_y;
        }

        float chassis_spin = transmitter.get_wheel() * 25;
        float pitch_target = 1.57
            + -transmitter.get_r_stick_y() * 0.3
            + transmitter_pos_y
            + vtm_pos_y;
        float yaw_target = -transmitter.get_r_stick_x() * 1.5
            - transmitter_pos_x
            - vtm_pos_x;
        float fly_wheel_target = (transmitter.get_r_switch() == 1 || transmitter.get_r_switch() == 3) ? 18 : 0; //m/s
        float feeder_target = (((transmitter.get_l_mouse_button() || ref.ref_data.kbm_interaction.button_left) && transmitter.get_r_switch() != 2) || transmitter.get_r_switch() == 1) ? 10 : 0;

        // set manual controls
        target_state[0][0] = chassis_pos_x;
        target_state[0][1] = chassis_vel_x;
        target_state[1][0] = chassis_pos_y;
        target_state[1][1] = chassis_vel_y;
        target_state[2][1] = chassis_spin;
        target_state[3][0] = yaw_target;
        target_state[3][1] = 0;
        target_state[4][0] = pitch_target;
        target_state[4][1] = 0;

        target_state[5][1] = fly_wheel_target;
        target_state[6][1] = feeder_target;
        target_state[7][0] = 1;

        // if the left switch is all the way down use Hive controls
        if (transmitter.get_l_switch() == 2) {
            incoming->get_target_state(target_state);
            // if you just switched to hive controls, set the reference to the current state
            if (hive_toggle) {
                governor.set_reference(temp_state);
                hive_toggle = false;
            }
        }

        // when in teensy control mode reset hive toggle
        if (transmitter.get_l_switch() == 3) {
            if (!hive_toggle) {
                pos_offset_x = temp_state[0][0];
                pos_offset_y = temp_state[1][0];
            }
            hive_toggle = true;
        }

        // read sensors
        estimator_manager.read_sensors();

        // override temp state if needed
        if (incoming->get_hive_override_request() == 1) {
            incoming->get_hive_override_state(hive_state_offset);
            memcpy(temp_state, hive_state_offset, sizeof(hive_state_offset));
        }

        // step estimates and construct estimated state
        estimator_manager.step(temp_state, temp_micro_state, incoming->get_hive_override_request());

        // if first loop set target state to estimated state
        if (count_one == 0) {
            temp_state[7][0] = 0;
            governor.set_reference(temp_state);
            count_one++;
        }

        // reference govern
        governor.set_estimate(temp_state);
        governor.step_reference(target_state, config->governor_types);
        governor.get_reference(temp_reference);

        // generate motor outputs from controls
        controller_manager.step(temp_reference, temp_state, temp_micro_state);

        // construct sensor data packet
        SensorData sensor_data;

        // set transmitter raw data
        //memcpy(sensor_data.raw + SENSOR_TRANSMITTER_OFFSET, transmitter.get_raw(), TRANSMITTER_PACKET_SIZE);

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

        bool is_slow_loop = false;

        // check whether this was a slow loop or not
	    float dt = stall_timer.delta();
        if (dt > 0.002) { 
            // zero the can bus just in case
	    	can.zero();
		
	    	Serial.printf("Slow loop with dt: %f\n", dt);
            // mark this as a slow loop to trigger safety mode
	    	is_slow_loop = true;
	    }
        
        //  SAFETY MODE
        if (transmitter.is_connected() && (transmitter.get_l_switch() == 2 || transmitter.get_l_switch() == 3) && config_layer.is_configured() && !is_slow_loop) {
            // SAFETY OFF
            can.write();
        } else {
            // SAFETY ON
            // TODO: Reset all controller integrators here
            can.zero();
        }

        // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        loopc++;

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
    }
    
    return 0;
}
