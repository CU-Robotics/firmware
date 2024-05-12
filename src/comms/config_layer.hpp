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
static const std::map<std::string, int> yaml_section_id_mappings = {
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

    void get_config_packets(CommsPacket packets[MAX_CONFIG_PACKETS]){
        memcpy(packets, config_packets, sizeof(packets));
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

    float gains[NUM_MOTORS][NUM_CONTROLLER_LEVELS][NUM_GAINS];;
    int assigned_states[NUM_ESTIMATORS][STATE_LEN];
    int num_states_per_estimator[NUM_ESTIMATORS];
    float set_reference_limits[STATE_LEN][3][2];



    void fill_data(CommsPacket packets[MAX_CONFIG_PACKETS]) {
        int index = yaml_section_id_mappings.at("num_motors");
        CommsPacket packet = packets[index];

        for(int i = 0; i < MAX_CONFIG_PACKETS; i++){
            if(packets[i].raw[1] == yaml_section_id_mappings.at("num_gains")){
                memcpy(&num_gains, packets[i].raw, sizeof(float));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("num_controller_levels")){
                memcpy(&num_controller_levels, packets[i].raw, sizeof(float));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("num_sensors")){
                memcpy(&num_sensors, packets[i].raw, sizeof(float));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("kinematics_p")){
                memcpy(kinematics_p, packets[i].raw, sizeof(kinematics_p));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("kinematics_v")){
                memcpy(kinematics_v, packets[i].raw, sizeof(kinematics_v));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("gains")){
                memcpy(gains, packets[i].raw, sizeof(gains));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("assigned_states")){
                memcpy(assigned_states, packets[i].raw, sizeof(assigned_states));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("num_states_per_estimator")){
                memcpy(num_states_per_estimator, packets[i].raw, sizeof(num_states_per_estimator));
            }
            if(packets[i].raw[1] == yaml_section_id_mappings.at("reference_limits")){
                memcpy(set_reference_limits, packets[i].raw, sizeof(set_reference_limits));
            }
        }
    }


};

#endif