#ifndef CONFIG_LAYER
#define CONFIG_LAYER

#include "usb_hid.hpp"
#include "../controls/controller.hpp"

#include <map>
#include <string>
#define CONFIG_LAYER_DEBUG

#define NUM_SENSOR_VALUES 12
#define NUM_SENSORS 16

/// @brief arbitrary cap on config packets that can be received (make sure it's enough)
const int MAX_CONFIG_PACKETS = 64;

/// @brief map section names to YAML section IDs
static const std::map<std::string, u_int8_t> yaml_section_id_mappings = {
    {"robot", 0},
    {"estimator_info", 1},
    {"controller_info", 2},
    {"gains", 3},
    {"motor_info", 4},
    {"gear_ratios", 5},
    {"sensor_info", 6},
    {"reference_limits", 7},
    {"governor_types", 8},
};

/// @brief struct to hold configuration data
struct Config {
    /// @brief fill all config data from packets
    /// @param packets CommsPacket array filled with data from yaml
    /// @param sizes Number of sections for each section
    void fill_data(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]);
    
    //check yaml for more details on values

    /// @brief robot id
    float robot;

    /// @brief matrix that defines type and neccessary values for each sensor
    float sensor_info[NUM_SENSORS][NUM_SENSOR_VALUES];

    /// @brief gains matrix
    float gains[NUM_ROBOT_CONTROLLERS][NUM_GAINS];
    /// @brief gear ratio matrix
    float gear_ratios[NUM_ROBOT_CONTROLLERS][NUM_MOTORS];

    /// @brief matrix that contains the type, physical id, and physical bus of each motor
    int motor_info[NUM_MOTORS][3];
    /// @brief reference limits matrix
    float set_reference_limits[STATE_LEN][3][2];
    
    /// @brief the estimator id's and info
    float estimator_info[NUM_ESTIMATORS][STATE_LEN + 1];
    /// @brief controller id's and info
    float controller_info[NUM_ROBOT_CONTROLLERS][NUM_MOTORS + 1];
    /// @brief governor types
    float governor_types[STATE_LEN];

private:
    /// @brief keep track of past index for when there are multiple packets for a section
    uint16_t index = 0;

};

/// @brief Handle seeking and reading configuration packets coming from khadas
class ConfigLayer {
private:
    /// @brief array to save config packets
    CommsPacket config_packets[MAX_CONFIG_PACKETS];

    /// @brief array to store number of subsections per YAML section
    uint8_t subsec_sizes[MAX_CONFIG_PACKETS] = { 0 };

    /// @brief number of YAML sections
    uint16_t num_sec;

    /// @brief current YAML section we are seeking
    int seek_sec = -1;

    /// @brief current YAML subsection we are seeking
    int seek_subsec = 0;

    /// @brief size counter
    int index = 0;

    /// @brief flag indicating if all config packets have been received
    bool configured = false;

    /// @brief a local instance of the config data
    Config config;

public:
    /// @brief default constructor
    ConfigLayer() { }

    /// @brief Block until all config packets are read, processed, and set within the returned Config object
    /// @param comms pointer to the HID comms layer for grabbing config packets
    /// @return a const pointer const config object holding all the data within the config yaml
    /// @note its double const so its enforced as a read-only object
    const Config* const configure(HIDLayer* comms);

    /// @brief check incoming packet from the comms layer and update outgoing packet accordingly to request next config packet
    /// @param in incoming comms packet
    /// @param out outgoing comms packet to write config requests to
    void process(CommsPacket *in, CommsPacket *out);

    /// @brief return configured flag (check if all config packets have been received)
    /// @return the configured flag
    bool is_configured() { return configured; }

    /// @brief get config and size arrays
    /// @param packets return array of packets
    /// @param sizes return array of sizes
    void get_config_packets(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]){
        memcpy(packets, config_packets, sizeof(CommsPacket) * MAX_CONFIG_PACKETS);
        memcpy(sizes, subsec_sizes, sizeof(uint8_t) * MAX_CONFIG_PACKETS);
    }
};

extern Config config;

#endif