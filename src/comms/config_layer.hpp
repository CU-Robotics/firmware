#ifndef CONFIG_LAYER
#define CONFIG_LAYER

#include "usb_hid.hpp"
#include "../controls/state.hpp"
#include "../controls/controller_manager.hpp"

#include <map>
#include <string>
#define CONFIG_LAYER_DEBUG

/// @brief arbitrary cap on config packets that can be received (make sure it's enough)
const int MAX_CONFIG_PACKETS = 64;

/// @brief map section names to YAML section IDs
static const std::map<std::string, u_int8_t> yaml_section_id_mappings = {
    {"robot", 0},
    {"pitch_angle_at_yaw_imu_calibration", 1},
    {"encoder_offsets", 2},
    {"yaw_axis_vector", 3},
    {"pitch_axis_vector", 4},
    {"default_gimbal_starting_angles", 5},
    {"default_chassis_starting_angles", 6},
    {"length_of_barrel_from_pitch_axis", 7},
    {"height_of_pitch_axis", 8},
    {"height_of_camera_above_barrel", 9},
    {"num_sensors", 10},
    {"estimators", 11},
    {"kinematics_p", 12},
    {"kinematics_v", 13},
    {"reference_limits", 14},
    {"controller_types", 15},
    {"gains", 16},
    {"num_states_per_estimator", 17},
    {"assigned_states", 18},
    {"switcher_values", 19},
    {"drive_conversion_factors", 20},
    {"governor_types", 21},
    {"odom_values", 22}
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

/// @brief struct to hold configuration data
struct Config {
    //check yaml for more details on values

    /// @brief number of motors
    float num_motors;
    /// @brief number of gains
    float num_gains;
    /// @brief number of controller levels
    float num_controller_levels;
    /// @brief Encoder offsets for each encoder
    float encoder_offsets[16]; 
    /// @brief number of sensors 
    float num_sensors[16];
    /// @brief position kinematics matrix
    float kinematics_p[NUM_MOTORS][STATE_LEN];
    /// @brief velocity kinematics matrix
    float kinematics_v[NUM_MOTORS][STATE_LEN];

    /// @brief gains matrix
    float gains[NUM_MOTORS][NUM_CONTROLLER_LEVELS][NUM_GAINS];
    /// @brief assigned states matrix
    float assigned_states[NUM_ESTIMATORS][STATE_LEN];
    /// @brief number of states per estimator
    float num_states_per_estimator[NUM_ESTIMATORS];
    /// @brief reference limits matrix
    float set_reference_limits[STATE_LEN][3][2];

    /// @brief governor types
    float estimators[NUM_ESTIMATORS];

    /// @brief gyro readings of imu when you spin yaw
    float yaw_axis_vector[3];
    /// @brief gyro readings of imu when you spin pitch
    float pitch_axis_vector[3];
    /// @brief default gimbal starting angles
    float default_gimbal_starting_angles[3];
    /// @brief default chassis starting angles
    float default_chassis_starting_angles[3];
    /// @brief controller types
    float controller_types[NUM_MOTORS][NUM_CONTROLLER_LEVELS];
    /// @brief values for chassis kinematics/dynamics
    float drive_conversion_factors[2];
    /// @brief what pitch angle we have when the the imu calibrates
    float pitch_angle_at_yaw_imu_calibration;
    /// @brief governor types
    float governor_types[STATE_LEN];
    /// @brief odom placement values
    float odom_values[3];
    /// @brief switcher placement values
    float switcher_values[2];

    /// @brief fill all config data from packets
    /// @param packets CommsPacket array filled with data from yaml
    /// @param sizes Number of sections for each section
    void fill_data(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]) {
        for(int i = 0; i < MAX_CONFIG_PACKETS; i++){
            uint8_t id = packets[i].get_id();
            uint8_t subsec_id = *reinterpret_cast<uint8_t*>(packets[i].raw + 2);
            uint16_t sub_size = *reinterpret_cast<uint16_t*>(packets[i].raw + 6);

            if(subsec_id == 0) index = 0;

            Serial.printf("id: %d, subsec_id: %d, sub_size: %d\n", id, subsec_id, sub_size);
            Serial.println();

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
                size_t linear_index = index / sizeof(float);
                size_t i1 = linear_index / STATE_LEN;
                size_t i2 = linear_index % STATE_LEN;
                memcpy(&assigned_states[i1][i2], packets[i].raw + 8, sub_size);
                index+=sub_size;
            }
            if(id == yaml_section_id_mappings.at("num_states_per_estimator")){
                memcpy(num_states_per_estimator, packets[i].raw + 8, sub_size);
            }
            if(id == yaml_section_id_mappings.at("reference_limits")){
                size_t linear_index = index / sizeof(float);
                size_t i1 = linear_index / (STATE_LEN * 3 * 2);
                size_t i2 = (linear_index % (STATE_LEN * 3 * 2)) / (3 * 2);
                size_t i3 = (linear_index % (STATE_LEN * 3 * 2)) % (3 * 2);
                memcpy(&set_reference_limits[i1][i2][i3], packets[i].raw + 8, sub_size);
                index+=sub_size;
                Serial.printf("indices: %d, %d, %d\n", i1, i2, i3);
            }
            if(id == yaml_section_id_mappings.at("yaw_axis_vector")){
                memcpy(yaw_axis_vector, packets[i].raw + 8,sub_size);
            }
            if(id == yaml_section_id_mappings.at("pitch_axis_vector")){
                memcpy(pitch_axis_vector, packets[i].raw + 8, sub_size);
            }
            if(id == yaml_section_id_mappings.at("default_gimbal_starting_angles")){
                memcpy(default_gimbal_starting_angles, packets[i].raw + 8, sub_size);
            }
            if(id == yaml_section_id_mappings.at("default_chassis_starting_angles")){
                memcpy(default_chassis_starting_angles, packets[i].raw + 8, sub_size);
            }
            if(id == yaml_section_id_mappings.at("controller_types")){
                memcpy(controller_types, packets[i].raw + 8, sub_size);
            }
            if(id == yaml_section_id_mappings.at("pitch_angle_at_yaw_imu_calibration")){
                memcpy(&pitch_angle_at_yaw_imu_calibration, packets[i].raw + 8, sub_size);
            }
            if(id==yaml_section_id_mappings.at("governor_types")){
                memcpy(governor_types, packets[i].raw + 8, sub_size);
            }
            if(id==yaml_section_id_mappings.at("drive_conversion_factors")){
                memcpy(drive_conversion_factors, packets[i].raw + 8, sub_size);
            }
            if(id==yaml_section_id_mappings.at("estimators")){
                memcpy(estimators, packets[i].raw + 8, sub_size);
                Serial.println(sub_size);
                Serial.println(estimators[0]);
            }
            if(id==yaml_section_id_mappings.at("odom_values")){
                memcpy(odom_values, packets[i].raw + 8, sub_size);
            }
            if(id==yaml_section_id_mappings.at("switcher_values")){
                memcpy(switcher_values, packets[i].raw + 8, sub_size);
            }
            if(id==yaml_section_id_mappings.at("num_sensors")){
                memcpy(&num_sensors, packets[i].raw + 8, sub_size);
            }
            if(id==yaml_section_id_mappings.at("encoder_offsets")){
                memcpy(encoder_offsets, packets[i].raw + 8, sub_size);
            }
        }
    }

    private: 
        /// @brief keep track of past index for when there are multiple packets for a section
        uint16_t index = 0;

};

#endif