#include <cstdint>
#include "comms/data/comms_data.hpp"
#include "sensor_constants.hpp"
#include "src/sensors/d200.hpp"

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

/// @brief Structure for the TOF (Time-of-Flight) sensor.
struct TOFSensorData : Comms::CommsData {
    TOFSensorData() : CommsData(Comms::TypeLabel::TOFSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(TOFSensorData)) { }
    /// Sensor ID.
    uint8_t id;
    /// Latest distance measurement.
    uint16_t latest_distance;
};

/// @brief Structure for the LiDAR sensor.
struct LidarSensorData : Comms::CommsData {
    LidarSensorData() : CommsData(Comms::TypeLabel::LidarSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LidarSensorData)) { }
    /// Sensor ID.
  uint8_t id;
  /// Index of the current data packet.
  int current_packet;
  /// Calibration data.
  D200Calibration cal;
  /// Array of cached data packets.
  LidarDataPacketSI packets[D200_NUM_PACKETS_CACHED];

};

/// @brief Structure for the DR16
struct DR16Data : Comms::CommsData {
    DR16Data() : CommsData(Comms::TypeLabel::DR16Data, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(DR16Data)) { }
    /// Sensor ID.
    uint8_t id;
    /// mouse x velocity
    int16_t mouse_x;
    /// mouse y velocity
    int16_t mouse_y;
    /// left mouse button status
    bool l_mouse_button;
    /// right mouse button status
    bool r_mouse_button;

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
        uint16_t keys;
        struct {
            unsigned w     : 1;
            unsigned s     : 1;
            unsigned a     : 1;
            unsigned d     : 1;
            unsigned shift : 1;
            unsigned ctrl  : 1;
            unsigned q     : 1;
            unsigned e     : 1;
            unsigned r     : 1;
            unsigned f     : 1;
            unsigned g     : 1;
            unsigned z     : 1;
            unsigned x     : 1;
            unsigned c     : 1;
            unsigned v     : 1;
            unsigned b     : 1;
        };
    };
};

