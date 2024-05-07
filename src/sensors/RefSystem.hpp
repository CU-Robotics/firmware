#ifndef REF_SYSTEM_HPP
#define REF_SYSTEM_HPP

#include "Arduino.h"

#include "RefSystemPacketDefs.hpp"

/// @brief Time (in us) between packet writes
constexpr uint32_t REF_MAX_PACKET_DELAY = 40000;
/// @brief Maximum number of bytes that is allowed to be sent within a second. Includes Ref header/tail
constexpr uint32_t REF_MAX_BAUD_RATE = 3720;
/// @brief Maximum number of inter robot packets should be able to be stored.
constexpr uint32_t REF_MAX_COMM_BUFFER_SIZE = 5;

/// @brief Game Staus packet offset for comms
constexpr uint32_t REF_COMMS_GAME_STATUS_OFFSET = 0;
/// @brief Game Result packet offset for comms
constexpr uint32_t REF_COMMS_GAME_RESULT_OFFSET = 11;
/// @brief Game Robot HP packet offset for comms
constexpr uint32_t REF_COMMS_GAME_ROBOT_HP = 12;
/// @brief Event Data packet offset for comms
constexpr uint32_t REF_COMMS_EVENT_DATE = 44;
/// @brief Projectile Supplier Status packet offset for comms
constexpr uint32_t REF_COMMS_PROJECTILE_SUPPLIER_STATUS = 48;
/// @brief Referee Warning packet offset for comms
constexpr uint32_t REF_COMMS_REFEREE_WARNING = 52;
/// @brief Dart Status packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_PERFORMANCE = 55;
/// @brief Robot Power Heat packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_POWER_HEAT = 68;
/// @brief Robot Position packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_POSITION = 84;
/// @brief Robot Buff packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_BUFF = 100;
/// @brief Damage Status packet offset for comms
constexpr uint32_t REF_COMMS_DAMAGE_STATUS = 106;
/// @brief Launching Status packet offset for comms
constexpr uint32_t REF_COMMS_LAUNCHING_STATUS = 107;
/// @brief Projectile Allowance packet offset for comms
constexpr uint32_t REF_COMMS_PROJECTILE_ALLOWANCE = 114;
/// @brief RFID Status packet offset for comms
constexpr uint32_t REF_COMMS_RFID_STATUS = 120;
/// @brief KBM Interaction packet offset for comms
constexpr uint32_t REF_COMMS_KBM_INTERACTION = 124;
/// @brief End of the Ref Data packet for comms
constexpr uint32_t REF_COMMS_END = 136;

/// @brief The serial line for the MCM
#define MCM_SERIAL (Serial2)
/// @brief The serial line for the VTM
#define VTM_SERIAL (Serial7)

/// @brief Generates a 1-byte CRC
/// @param data data array
/// @param length size of the data array
/// @return the 8-bit CRC
uint8_t generateCRC8(const uint8_t* data, uint32_t length);
/// @brief Generates a 2-byte CRC
/// @param data data array
/// @param length size of the data array
/// @return the 16-bit CRC
uint16_t generateCRC16(const uint8_t* data, uint32_t length);
const uint8_t CRC8Lookup[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
const uint16_t CRC16Lookup[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/// @brief Wrapper class to send and receive packets from the Referee System
/// @see https://rm-static.djicdn.com/tem/17348/RoboMaster%20Referee%20System%20Serial%20Port%20Protocol%20Appendix%20V1.6.1%EF%BC%8820240126%EF%BC%89.pdf
class RefSystem {
public:
    /// @brief Default constructor. Set to do nothing
    RefSystem();

    /// @brief Initializes the RefSystem. Sets up the Serial connection
    void init();

    /// @brief Reads the incoming data from the Referee System and sets the data into ref_data
    void read();

    /// @brief Send a pre-constructed packet to Ref
    /// @param packet Byte array of the packet to be sent
    /// @param length The total size of the packet, including header/tail
    /// @note Re-computes the CRC, so no need to do it yourself
    void write(uint8_t* packet, uint8_t length);

    /// @brief Generate a byte array for all ref data to be sent over comms
    /// @param output_array Byte array to store the data
    /// @note Only sends some packets, not all
    void get_data_for_comms(uint8_t output_array[180]);

private:
    /// @brief Helper function: Reads and stores frame header
    /// @param serial Serial port to read from
    /// @param frame Frame reference to fill
    /// @param raw_buffer Buffer to read to
    /// @param buffer_index Index into the buffer
    /// @return Whether the read was successful or not
    bool read_frame_header(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame);

    /// @brief Helper function: Reads and stores frame command ID
    /// @param serial Serial port to read from
    /// @param frame Frame reference to fill
    /// @param raw_buffer Buffer to read to
    /// @param buffer_index Index into the buffer    
    /// @return Whether the read was successful or not
    bool read_frame_command_ID(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame);

    /// @brief Helper function: Reads and stores frame data
    /// @param serial Serial port to read from
    /// @param frame Frame reference to fill
    /// @param raw_buffer Buffer to read to
    /// @param buffer_index Index into the buffer    
    /// @return Whether the read was successful or not
    bool read_frame_data(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame);

    /// @brief Helper function: Reads and stores frame tail
    /// @param serial Serial port to read from
    /// @param frame Frame reference to fill
    /// @param raw_buffer Buffer to read to
    /// @param buffer_index Index into the buffer    
    /// @return Whether the read was successful or not
    bool read_frame_tail(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame);

    /// @brief Helper function: sets the data in a frame to the ref_data struct
    /// @param frame Frame to read from
    /// @param raw_buffer Buffer to read from
    void set_ref_data(Frame& frame, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2]);

    /// @brief Get the current outgoing sequence. Used in sending frames
    /// @return The next sequence
    inline uint8_t get_seq() noexcept { seq++;  return seq; }

private:
    /// @brief Current sequence number. Used to send packets
    uint8_t seq = 0;
    
    // VTM

    /// @brief Buffer to store frames-in-progress (for vtm packets)
    /// @note This is * 2 to allow buffer space for the largest frame
    uint8_t vtm_raw_buffer[REF_MAX_PACKET_SIZE * 2] = { 0 };

    /// @brief Index into the raw_buffer (for vtm packets)
    uint16_t vtm_buffer_index = 0;

    /// @brief Whether the header has been successfully read (for vtm packets)
    bool vtm_header_read = false;
    /// @brief Whether the command ID has been successfully read (for vtm packets)
    bool vtm_command_ID_read = false;
    /// @brief Whether the data has been successfully read (for vtm packets)
    bool vtm_data_read = false;

    /// @brief Current frame being read (for vtm packets)
    Frame vtm_curr_frame{};

    // MCM

    /// @brief Buffer to store frames-in-progress (for mcm packets)
    /// @note This is * 2 to allow buffer space for the largest frame
    uint8_t mcm_raw_buffer[REF_MAX_PACKET_SIZE * 2] = { 0 };

    /// @brief Index into the raw_buffer (for mcm packets)
    uint16_t mcm_buffer_index = 0;

    /// @brief Whether the header has been successfully read (for mcm packets)
    bool mcm_header_read = false;
    /// @brief Whether the command ID has been successfully read (for mcm packets)
    bool mcm_command_ID_read = false;
    /// @brief Whether the data has been successfully read (for mcm packets)
    bool mcm_data_read = false;

    /// @brief Current frame being read (for mcm packets)
    Frame mcm_curr_frame{};

public:
    /// @brief Number of inter-robot packets sent
    uint32_t packets_sent = 0;
    /// @brief Number of inter-robot packets received
    uint32_t packets_received = 0;
    /// @brief Number of packets that failed to be read properly
    uint32_t packets_failed = 0;

    /// @brief Current count of bytes sent since last reset
    uint16_t bytes_sent = 0;

    /// @brief struct to store all ref data
    RefData ref_data{};
};

extern RefSystem ref;

#endif // REF_SYSTEM_HPP