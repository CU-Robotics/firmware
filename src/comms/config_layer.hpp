#ifndef CONFIG_LAYER
#define CONFIG_LAYER

#include "usb_hid.hpp"
#include "../controls/state.hpp"
#define CONFIG_LAYER_DEBUG

/// @brief arbitrary cap on config packets that can be received (make sure it's enough)
const int MAX_CONFIG_PACKETS = 64;

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
    float num_sensors;
    float encoder_offsets[16];  
    float num_sensors[16];
    float kinematics_p[NUM_MOTORS][STATE_LEN];
    float kinematics_v[NUM_MOTORS][STATE_LEN];

    float num_states_per_estimator[NUM_ESTIMATORS];
    float assigned_states[NUM_ESTIMATORS][STATE_LEN];
    float gains[NUM_MOTORS][NUM_CONTROLLER_LEVELS][NUM_GAINS] = { 0 };
    int assigned_states[NUM_ESTIMATORS][STATE_LEN] = { 0 };
    int num_states_per_estimator[NUM_ESTIMATORS] = { 0 };
    float set_reference_limits[STATE_LEN][3][2] = { 0 };


    /// @brief map section names to YAML section IDs
    static const std::map<std::string, int> yaml_section_id_mappings = {
        {"robot", 0},
        {"num_motors", 1},
        {"num_gains", 2},
        {"num_controller_levels", 3},
        {"num_sensors", 4},
        {"estimators", 5},
        {"kinematics_p", 6},
        {"kinematics_v", 7},
        {"reference_limits", 8},
        {"estimator_states", 9},
        {"controller_types", 10},
        {"gains", 11},
        {"num_states_per_estimator", 12},
        {"assigned_states", 13} 
    };

    void fill_data(CommsPacket packets[MAX_CONFIG_PACKETS]) {
        int index = yaml_section_id_mappings.at("num_motors");
        CommsPacket packet = packets[index];
        memcpy(&num_motors, packet.raw, sizeof(float));
    }


}

#endif