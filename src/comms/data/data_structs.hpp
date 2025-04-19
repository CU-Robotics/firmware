#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

/// @brief Data struct for testing purposes
struct TestData : Comms::CommsData {
    TestData() : Comms::CommsData(Comms::TypeLabel::TestData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TestData)) {}
    /// @brief x value
    float x = 1.f;
    /// @brief y value
    float y = 2.f;
    /// @brief z value
    float z = 3.f;
    /// @brief w value
    uint32_t w = 0x98765432;
};

/// @brief Data struct for testing purposes
struct BigTestData : Comms::CommsData {
    BigTestData() : Comms::CommsData(Comms::TypeLabel::BigTestData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(BigTestData)) {}

    /// @brief a bunch of data
    float blah[128] = { 0 };
};

/// @brief Structure for the buff encoder sensor.
struct BuffEncoderData : Comms::CommsData {
    BuffEncoderData() : CommsData(Comms::TypeLabel::BuffEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(BuffEncoderData)) { }
    /// Sensor ID.
    uint8_t id;
    /// Measured angle.
    float m_angle;
};

/// @brief Structure for the Rev encoder sensor.
struct RevSensorData : Comms::CommsData {
    RevSensorData() : CommsData(Comms::TypeLabel::RevEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(RevSensorData)) { }
	/// Sensor ID.
	uint8_t id;
	/// Encoder ticks.
	int ticks;
	/// Angle in radians.
	float radians;
};

/// @brief Structure for the ICM sensor.
struct ICMSensorData : Comms::CommsData {
    ICMSensorData() : CommsData(Comms::TypeLabel::ICMSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(ICMSensorData)) { }
    /// Sensor ID.
    uint8_t id;
    /// Acceleration in X-axis.
    float accel_X;
    /// Acceleration in Y-axis.
    float accel_Y;
    /// Acceleration in Z-axis.
    float accel_Z;
    /// Gyroscope reading in X-axis.
    float gyro_X;
    /// Gyroscope reading in Y-axis.
    float gyro_Y;
    /// Gyroscope reading in Z-axis.
    float gyro_Z;
    /// Temperature reading.
    float temperature;
};

/// @brief Structure for the TOF (Time-of-Flight) sensor.
struct TOFSensorData : Comms::CommsData {
    TOFSensorData() : CommsData(Comms::TypeLabel::TOFSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(TOFSensorData)) { }
    /// Sensor ID.
    uint8_t id;
    /// Latest distance measurement.
    uint16_t latest_distance;
};

/// @brief data for a singular LiDAR packet (SI units)
struct LidarDataPacketSI : Comms::CommsData {
    LidarDataPacketSI() : CommsData(Comms::TypeLabel::LidarDataPacketSI, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(LidarDataPacketSI)) { }

    /// @brief number of points per packet
    static constexpr uint32_t D200_POINTS_PER_PACKET = 12;

    /// @brief the id of the lidar module
    uint8_t id = 0;

    /// @brief speed of lidar module (rad/s)
    float lidar_speed = 0;
  
    /// @brief start angle of measurements (rad)
    float start_angle = 0;
    
    // I made these separate arrays because it reduced the packet size due to alignment issues
    // goes from 96 bytes for these two arrays to 60
    /// @brief distances (m)
    float distances[D200_POINTS_PER_PACKET] = {0};

    /// @brief intensity of measurement. units are ambiguous (not documented), but in general "the higher the intensity, the larger the signal strength value"
    uint8_t intensities[D200_POINTS_PER_PACKET] = {0};
    
    /// @brief end angle of measurements (rad)
    float end_angle = 0;
  
    /// @brief timestamp of measurements, from the lidar, calibrated (s)
    float timestamp = 0;

    /// @brief teensy time of when the packet was received, (s)
    float sample_time = 0;

    /// @brief the angle difference between each lidar point (rad)
    /// @return the angle difference between each lidar point (rad)
    float angle_diff() const { return (end_angle - start_angle) / D200_POINTS_PER_PACKET; }

    /// @brief the time it takes for the lidar to sweep from the start to end angles (s)
    /// @return the time it takes for the lidar to sweep from the start to end angles (s)
    float sweep_time() const { return (end_angle - start_angle) / lidar_speed; }

    /// @brief the yaw of the robot when the packet was received (rad)
    float yaw = 0;

    /// @brief the yaw velocity of the robot when the packet was received (rad/s)
    float yaw_velocity = 0;
};

/// @brief Structure for the DR16
struct DR16Data : Comms::CommsData {
    DR16Data() : CommsData(Comms::TypeLabel::DR16Data, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(DR16Data)) { }
    /// Sensor ID.
    uint8_t id;
    /// mouse x velocity
    int16_t mouse_x = 0;
    /// mouse y velocity
    int16_t mouse_y = 0;
    /// mouse z velocity
    int16_t mouse_z = 0;
    /// left mouse button status
    bool l_mouse_button = 0;
    /// right mouse button status
    bool r_mouse_button = 0;
    /// left switch status
    float l_switch = 0;
    /// right switch status
    float r_switch = 0;
    /// left stick x axis
    float l_stick_x = 0;
    /// left stick y axis
    float l_stick_y = 0;
    /// right stick x axis
    float r_stick_x = 0;
    /// right stick y axis
    float r_stick_y = 0;
    /// wheel
    float wheel = 0;

    /**
     * Usage example of how to acces the keys bitfield:
        DR16Data data;
        data.w = 1;          // Mark key 'w' as pressed
        if (data.s) {        // Check if key 's' is pressed
        // do something
        }
     * 
     */
    union {
        uint16_t raw = 0;
        struct {
            uint16_t w     : 1;
            uint16_t s     : 1;
            uint16_t a     : 1;
            uint16_t d     : 1;
            uint16_t shift : 1;
            uint16_t ctrl  : 1;
            uint16_t q     : 1;
            uint16_t e     : 1;
            uint16_t r     : 1;
            uint16_t f     : 1;
            uint16_t g     : 1;
            uint16_t z     : 1;
            uint16_t x     : 1;
            uint16_t c     : 1;
            uint16_t v     : 1;
            uint16_t b     : 1;
        } key;
    } keys;
};

// TODO: replace with kyle3's new state struct
/// @brief Structure for the full robot state including time
struct TempRobotState : Comms::CommsData {
    TempRobotState() : CommsData(Comms::TypeLabel::TempRobotState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TempRobotState)) { }
  
    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
};

/// @brief Full robot state, in the form of TargetState
struct TargetState : Comms::CommsData {
    TargetState() : CommsData(Comms::TypeLabel::TargetState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TargetState)) { }

    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
};

/// @brief Full robot state, in the form of EstimatedState
struct EstimatedState : Comms::CommsData {
    EstimatedState() : CommsData(Comms::TypeLabel::EstimatedState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(EstimatedState)) { }
  
    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
};

/// @brief Full robot state, in the form of OverrideState
struct OverrideState : Comms::CommsData {
    OverrideState() : CommsData(Comms::TypeLabel::OverrideState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(OverrideState)) { }
  
    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
    /// @brief Whether this request is active or not
    bool active = false;
};

// TODO: make this ethernet capable
/// @brief Section of a config packet
struct ConfigSection : Comms::CommsData {
    ConfigSection() : CommsData(Comms::TypeLabel::ConfigSection, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(ConfigSection)) { }

    /// @brief filler byte
    uint8_t filler = 0xff;
    /// @brief Section ID
    int8_t section_id = 0;
    /// @brief Subsection ID
    int8_t subsection_id = 0;
    /// @brief Info bit, stores the config request bit
    uint8_t info_bit = 0;

    /// @brief Size of the whole section
    uint16_t section_size = 0;
    /// @brief Size of the subsection
    uint16_t subsection_size = 0;

    /// @brief Raw config data
    uint8_t raw[1000] = { 0 };
};

// TODO: make this nice
/// @brief Ref data
struct CommsRefData : Comms::CommsData {
    CommsRefData() : CommsData(Comms::TypeLabel::CommsRefData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(CommsRefData)) { }

    /// @brief Raw data
    uint8_t raw[180] = { 0 };
};

#endif // DATA_STRUCTS_HPP