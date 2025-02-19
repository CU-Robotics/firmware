#ifndef D200_H
#define D200_H

// Arduino library
#include <Arduino.h>
#include <HardwareSerial.h>

// development manual
// https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf

// specs
// https://www.waveshare.com/d200-lidar-kit.htm

/// @brief table of constants for computing CRC8 checksums
const uint8_t CRC_TABLE[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
  0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
  0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
  0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
  0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
  0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
  0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
  0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
  0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
  0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
  0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
  0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
  0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
  0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
  0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
  0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
  0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
  0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
  0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
  0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
  0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
  0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

/// @brief size of each data packet sent over comms
const int D200_PAYLOAD_SIZE = 77;

/// @brief points per D200 data packet
const int D200_POINTS_PER_PACKET = 12;

/// @brief number of packets stored teensy-side
const int D200_NUM_PACKETS_CACHED = 2;

/// @brief baud rate of LiDAR module
const int D200_BAUD = 230400;

/// @brief start character for packet
const int D200_START_CHAR = 0x54;

/// @brief frame character for data packet
const int D200_FRAME_CHAR = 0x2c;

/// @brief length of data packet (bytes)
const int D200_DATA_PACKET_LEN = 47;

/// @brief length of command packet (bytes)
const int D200_CMD_PACKET_LEN = 8;

/// @brief hard-coded start command
const uint8_t D200_START_CMD[] = { 0x54, 0xa0, 0x04, 0, 0, 0, 0, 0x5e };

/// @brief hard-coded stop command
const uint8_t D200_STOP_CMD[] = { 0x54, 0xa1, 0x04, 0, 0, 0, 0, 0x4a };

/// @brief default scanning speed (rad/s)
const float D200_DEFAULT_SPEED = (float)(6 * 360) * M_PI / 180.0;

/// @brief min specified scanning speed (rad/s)
const float D200_MIN_SPEED = (float)(2 * 360 + 1) * M_PI / 180.0;

/// @brief max specified scanning speed (rad/s)
const float D200_MAX_SPEED = (float)(8 * 360 - 1) * M_PI / 180.0;

/// @brief number of timestamp calibration packets. new packet read rate of 333 Hz = 1 packet / 3ms
const int D200_MAX_CALIBRATION_PACKETS = 333;

/// @brief max millis before lidar timestamp wraps (30s)
const int D200_TIMESTAMP_WRAP_LIMIT = 30000;

/*
/// @brief struct storing data from lidar data packet (native units).
struct LidarDataPacket {
  /// @brief speed of lidar module (deg/s)
  uint16_t lidar_speed = 0;
  
  /// @brief start angle of measurements (hundredths of deg)
  uint16_t start_angle = 0;
  
  /// @brief array of point measurements
  struct {
    /// @brief distance (mm)
    uint16_t distance;

    /// @brief intensity of measurement. units are ambiguous (not documented), but in general "the higher the intensity, the larger the signal strength value"
    uint8_t intensity = 0;
  } points[D200_POINTS_PER_PACKET];
  
  /// @brief end angle of measurements (hundredths of deg)
  uint16_t end_angle = 0;
  
  /// @brief timestamp of measurements, wraps after 30s (ms)
  uint16_t timestamp = 0;
}; */

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

/// @brief class for LiDAR driver
class D200LD14P {
  private:
    /// @brief default scanning speed (deg/s) (used internally)
    const uint16_t DEFAULT_SPEED = 6 * 360;

    /// @brief minimum specified scanning speed (deg/s) (used internally)
    const uint16_t MIN_SPEED = 2 * 360 + 1;

    /// @brief maximum specified scanning speed (deg/s) (used internally)
    const uint16_t MAX_SPEED = 8 * 360 - 1;

    /// @brief teensy-side cache of packets for comms to use. number of cached packets may be adjusted as needed
    LidarDataPacketSI packets[D200_NUM_PACKETS_CACHED];

    /// @brief index of current packet (wraps)
    int current_packet = 0;

    /// @brief serial object to read from
    HardwareSerial *port = nullptr;

    /// @brief assigned ID of this specific module
    uint8_t id;

    /// @brief timestamp calibration results
    D200Calibration cal;

    /// @brief utility for bitcasting float to 32bit unsigned integer
    /// @param f32 float to bitcat to a uint32_t
    /// @return uint32_t with same bits as passed float
    uint32_t bitcast_float(float f32);

    /// @brief compute CRC8 checksum for buffer
    /// @param buf pointer to buffer
    /// @param len length of buffer
    /// @return CRC8 checksum for buffer
    uint8_t calc_checksum(uint8_t *buf, int len);

  public:
    /// @brief constructor and initialization
    /// @param _port pointer to HardwareSerial object to read/write from
    /// @param _id id of the LiDAR object
    D200LD14P(HardwareSerial *_port, uint8_t _id);

    /// @brief set rotation the speed of the LiDAR
    /// @param speed desired rotation speed of LiDAR (rad/s)
    void set_speed(float speed);

    /// @brief start the LiDAR motor
    void start_motor();

    /// @brief stop the LiDAR motor
    void stop_motor();

    /// @brief read latest packet(s) from D200 module
    void read();

    /// @brief get the packet array
    /// @return pointer to start of packet array
    LidarDataPacketSI *get_packets() { return packets; }

    /// @brief get the index of the most recent packet
    /// @return index of latest packet
    int get_latest_packet_index() { return current_packet; }

    /// @brief get the most recently read (complete) packet
    /// @return the most recently read packet. if no packets have been read, this will return a zero-initialized packet
    LidarDataPacketSI get_latest_packet() { return packets[current_packet]; }

    /// @brief print the most recently read (complete) packet for debugging purposes
    void print_latest_packet();

    /// @brief flush the packet buffer
    void flush_packet_buffer();

    /// @brief export LiDAR data as byte array for comms. Exports the latest D200_NUM_PACKETS_CACHED packets, for a total size of D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE bytes per export.
    /// @param bytes byte array to write LiDAR data into
    void export_data(uint8_t bytes[D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE]);
};

#endif // D200_H