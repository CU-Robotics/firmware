#ifndef CONFIG_LAYER
#define CONFIG_LAYER

#include "usb_hid.hpp"
#include "../controls/controller.hpp"
#include "SDManager.hpp"

#include <map>
#include <string>
#define CONFIG_LAYER_DEBUG

// config err handler macros
#define CONFIG_RM_FAIL 0
#define CONFIG_TOUCH_FAIL 1
#define CONFIG_OPEN_FAIL 2
#define CONFIG_ID_MISMATCH 3

// define CONFIG_OFF_ROBOT macro when running off of real robot (testing firmware away from actual robot)
// #define CONFIG_OFF_ROBOT 

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
    {"odom_values", 22},
    {"encoder_pins", 23}
};

/// @brief struct to hold configuration data
struct Config {
    /// @brief fill all config data from packets
    /// @param packets CommsPacket array filled with data from yaml
    /// @param sizes Number of sections for each section
    void fill_data(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]);

    //check yaml for more details on values
    /// @brief robot id sent from hive
    float robot_id;
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
    /// @brief pin numbers on the teensy for the encoders
    float encoder_pins[2];

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

    /// @brief sd card object for interacting with config files
    SDManager sdcard;

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
    void process(CommsPacket* in, CommsPacket* out);

    /// @brief return configured flag (check if all config packets have been received)
    /// @return the configured flag
    bool is_configured() { return configured; }

    /// @brief get config and size arrays
    /// @param packets return array of packets
    /// @param sizes return array of sizes
    void get_config_packets(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]) {
        memcpy(packets, config_packets, sizeof(CommsPacket) * MAX_CONFIG_PACKETS);
        memcpy(sizes, subsec_sizes, sizeof(uint8_t) * MAX_CONFIG_PACKETS);
    }

    /// @brief check if SD card is available to load from, and wait for ref system initialization
    /// @param comms 
    void config_SD_init(HIDLayer* comms);

    /// @brief read packet data from SD card at /config.pack
    /// @param checksum variable to store checksum into (passed by reference in order to use outside of function)
    /// @return false if any errors were encountered during load procedure, true otherwise.
    bool config_SD_read_packets(uint64_t& checksum);

    /// @brief attempt to load configuration stored on sd card, assuming it exists
    /// @return true if successful, false otherwise
    bool sd_load();

    /// @brief attempt to store configuration from comms, only runs after comms is run 
    /// @return true if successful, false otherwise
    bool store_config();

    /// @brief compute sum of bytes of array (arr) of size n
    /// @param arr array whose data we take sum of
    /// @param n number of bytes in arr
    /// @return 64-bit sum of bytes in arr
    /// @note there is no need to do this as 64-bit (slow), but it will work for now
    uint64_t sd_checksum64(uint8_t* arr, uint64_t n);

    /// @brief handles errors during the configuration procedure from the SD card
    /// @param err_code error code to identify which behavior to execute
    /// @return false when error is unrecoverable or fails to recover, true when successfully recovers.
    bool CONFIG_ERR_HANDLER(int err_code);
};

#endif