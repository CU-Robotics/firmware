#ifndef CONFIG_LAYER
#define CONFIG_LAYER

#include "usb_hid.hpp"
#include "../controls/state.hpp"
#include "../controls/controller_manager.hpp"
#include "../controls/estimator_manager.hpp"
#include <map>
#include <string>
#define CONFIG_LAYER_DEBUG

/// @brief arbitrary cap on config packets that can be received (make sure it's enough)
const int MAX_CONFIG_PACKETS = 64;

/// @brief map section names to YAML section IDs
static const std::map<std::string, u_int8_t> yaml_section_id_mappings = {
    {"robot", 0},
    {"num_motors", 1},
    {"num_estimators", 2},
    {"num_gains", 3},
    {"num_controller_levels", 4},
    {"encoder_offsets", 5},
    {"yaw_axis_vector", 6},
    {"pitch_axis_vector", 7},
    {"defualt_gimbal_starting_angles", 8},
    {"defualt_chassis_starting_angles", 9},
    {"length_of_barrel_from_pitch_axis", 10},
    {"height_of_pitch_axis", 11},
    {"height_of_camera_above_barrel", 12},
    {"num_sensors", 13},
    {"estimators", 14},
    {"kinematics_p", 15},
    {"kinematics_v", 16},
    {"reference_limits", 17},
    {"controller_types", 18},
    {"gains", 19},
    {"num_states_per_estimator", 20},
    {"assigned_states", 21}
};

class ConfigLayer {
private:
    /// @brief array to save config packets
    CommsPacket config_packets[MAX_CONFIG_PACKETS];

    /// @brief flag indicating if all config packets have been received
    bool configured = false;

    /// @brief current YAML section we are seeking
    int seek_sec = -1;

    /// @brief current YAML subsection we are seeking
    int seek_subsec = 0;

    /// @brief number of YAML sections
    uint16_t num_sec;

    /// @brief array to store number of subsections per YAML section
    uint8_t subsec_sizes[MAX_CONFIG_PACKETS] = { 0 }; 

    int index = 0;

public:
    /// @brief default constructor
    ConfigLayer() {}

    /// @brief check incoming packet from the comms layer and update outgoing packet accordingly to request next config packet
    /// @param in incoming comms packet
    /// @param out outgoing comms packet to write config requests to
    void process(CommsPacket *in, CommsPacket *out);

    /// @brief return configured flag (check if all config packets have been received)
    /// @return the configured flag
    bool is_configured() { return configured; }

    void get_config_packets(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]){
        memcpy(packets, config_packets, sizeof(CommsPacket) * MAX_CONFIG_PACKETS);
        memcpy(sizes, subsec_sizes, sizeof(uint8_t) * MAX_CONFIG_PACKETS);
    }
};

struct Config {
    float num_motors;
    float num_gains;
    float num_controller_levels;
    float encoder_offsets[16];  
    float num_sensors[16];
    float kinematics_p[NUM_MOTORS][STATE_LEN];
    float kinematics_v[NUM_MOTORS][STATE_LEN];

    float gains[NUM_MOTORS][NUM_CONTROLLER_LEVELS][NUM_GAINS];
    float assigned_states[NUM_ESTIMATORS][STATE_LEN];
    float num_states_per_estimator[NUM_ESTIMATORS];
    float set_reference_limits[STATE_LEN][3][2];

    float yaw_axis_vector[2];
    float pitch_axis_vector[2];
    float defualt_gimbal_starting_angles[2];
    float defualt_chassis_starting_angles[2];
    float controller_types[NUM_MOTORS][NUM_CONTROLLER_LEVELS];

    void fill_data(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]) {
        int j = 0;

        for(int i = 0; i < MAX_CONFIG_PACKETS; i++){
            uint8_t id = packets[i].get_id();
            uint8_t subsec_id = *reinterpret_cast<uint8_t*>(packets[i].raw + 2);
            uint16_t sub_size = *reinterpret_cast<uint16_t*>(packets[i].raw + 6);

            if(subsec_id == 0) index = 0;

            if(id == yaml_section_id_mappings.at("num_gains")){
                memcpy(&num_gains, packets[i].raw + 8, sizeof(float));
            }
            if(id == yaml_section_id_mappings.at("num_controller_levels")){
                memcpy(&num_controller_levels, packets[i].raw + 8, sizeof(float));
            }
            if(id == yaml_section_id_mappings.at("num_sensors")){
                memcpy(&num_sensors, packets[i].raw + 8, sizeof(float));
            }
            if(id == yaml_section_id_mappings.at("kinematics_p")){
                size_t linear_index = index / sizeof(float);
                size_t i1 = linear_index / STATE_LEN;
                size_t i2 = linear_index % STATE_LEN;
                memcpy(&kinematics_p[i1][i2], packets[i].raw + 8, sub_size);
                index+=sub_size;
            }
            if(id == yaml_section_id_mappings.at("kinematics_v")){
                size_t linear_index = index / sizeof(float);
                size_t i1 = linear_index / STATE_LEN;
                size_t i2 = linear_index % STATE_LEN;
                memcpy(&kinematics_v[i1][i2], packets[i].raw + 8, sub_size);
                index+=sub_size;

            }
            if(id == yaml_section_id_mappings.at("gains")) {

                size_t linear_index = index / sizeof(float); 
                size_t i1 = linear_index / (NUM_CONTROLLER_LEVELS * NUM_GAINS);
                size_t i2 = (linear_index % (NUM_CONTROLLER_LEVELS * NUM_GAINS)) / NUM_GAINS;
                size_t i3 = (linear_index % (NUM_CONTROLLER_LEVELS * NUM_GAINS)) % NUM_GAINS;
                memcpy(&gains[i1][i2][i3], packets[i].raw + 8, sub_size);
                // Serial.println(index/sizeof(float));
                index+=sub_size;
            }
            if(id == yaml_section_id_mappings.at("assigned_states")){
                memcpy(assigned_states, packets[i].raw + 8, sizeof(assigned_states));
            }
            if(id == yaml_section_id_mappings.at("num_states_per_estimator")){
                memcpy(num_states_per_estimator, packets[i].raw + 8, sizeof(num_states_per_estimator));
            }
            if(id == yaml_section_id_mappings.at("reference_limits")){
                size_t linear_index = index / sizeof(float);
                size_t i1 = linear_index / (STATE_LEN * 3 * 2);
                size_t i2 = (linear_index % (STATE_LEN * 3 * 2)) / (3 * 2);
                size_t i3 = (linear_index % (STATE_LEN * 3 * 2)) % (3 * 2);
                memcpy(&set_reference_limits[i1][i2][i3], packets[i].raw + 8, sizeof(set_reference_limits));
                index+=sub_size;
                Serial.printf("indices: %d, %d, %d\n", i1, i2, i3);
            }
            if(id == yaml_section_id_mappings.at("yaw_axis_vector")){
                memcpy(yaw_axis_vector, packets[i].raw + 8, sizeof(yaw_axis_vector));
            }
            if(id == yaml_section_id_mappings.at("pitch_axis_vector")){
                memcpy(pitch_axis_vector, packets[i].raw + 8, sizeof(pitch_axis_vector));
            }
            if(id == yaml_section_id_mappings.at("defualt_gimbal_starting_angles")){
                memcpy(defualt_gimbal_starting_angles, packets[i].raw + 8, sizeof(defualt_gimbal_starting_angles));
            }
            if(id == yaml_section_id_mappings.at("defualt_chassis_starting_angles")){
                memcpy(defualt_chassis_starting_angles, packets[i].raw + 8, sizeof(defualt_chassis_starting_angles));
            }
            if(id == yaml_section_id_mappings.at("controller_types")){
                memcpy(controller_types, packets[i].raw + 8, sizeof(controller_types));
            }
        }

        // Serial.println(j);
    }

    void print() {
        Serial.println("Config data:");
        // for(int i = 0; i < NUM_MOTORS; i++){
        //     for(int j = 0; j < STATE_LEN; j++){
        //         Serial.printf("kinematics_v[%d][%d]: %f\n", i, j, kinematics_v[i][j]);
        //     }
        // }

        //Reference limits:

        for(int i = 0; i < STATE_LEN; i++){
            for(int j = 0; j < 3; j++){
                for(int k = 0; k < 2; k++){
                    Serial.printf("set_reference_limits[%d][%d][%d]: %f\n", i, j, k, set_reference_limits[i][j][k]);
                }
            }
        }

    }

    private: 
        uint16_t index = 0;

};

#endif