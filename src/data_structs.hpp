#include <cstdint>
#include "comms/data/comms_data.hpp"
#include "sensor_constants.hpp"

#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

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

/// @brief data for a LiDAR packet (SI units)
struct LidarDataPacketSI {
    /// @brief speed of lidar module (rad/s)
    float lidar_speed = 0;
  
    /// @brief start angle of measurements (rad)
    float start_angle = 0;
  
    /// @brief array of point measurements
    struct {
      /// @brief distance (m)
      float distance;
  
      /// @brief intensity of measurement. units are ambiguous (not documented), but in general "the higher the intensity, the larger the signal strength value"
      uint8_t intensity = 0;
    } points[D200_POINTS_PER_PACKET];
  
    /// @brief end angle of measurements (rad)
    float end_angle = 0;
  
    /// @brief timestamp of measurements, calibrated (s)
    float timestamp = 0;
  };
  
  /// @brief struct storing timestamp calibration results 
  struct D200Calibration {
    /// @brief how many packets are used for calibration
    int max_calibration_packets = D200_MAX_CALIBRATION_PACKETS;
  
    /// @brief how many calibration packets have been received
    int packets_recv = 0;
  
    /// @brief count of D200 timestamp wraps
    int num_wraps = 0;
  
    /// @brief previous lidar timestamp
    int prev_timestamp = -1;
  
    /// @brief sum of delta times for calibration packets
    int timestamp_delta_sum = 0;
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
    union KeyUnion{
        uint16_t keys;
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
        };
    };
};

#endif // DATA_STRUCTS_HPP