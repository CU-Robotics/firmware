#ifndef SERIAL_COMMS
#define SERIAL_COMMS

/**
* DETECT PLATFORM (LINUX/TEENSY4.1)
*/
#ifdef __linux__
#define LINUX
#endif

/**
* GLOBAL INCLUDES
*/
#include <cstdint> // integer types

#ifdef LINUX

/**
* LINUX INCLUDES
*/
#include <cstdio> // printf and family
#include <fcntl.h> // file IO
#include <unistd.h> // more file IO
#include <termios.h> // serial config
#include <errno.h> // error tracking
#include <err.h> // more error tracking
#include <sys/ioctl.h> // baud rate aliasing on Linux
#include <linux/serial.h> // baud rate aliasing on Linux

#else // TEENSY

/**
* TEENSY INCLUDES
*/
#include <Arduino.h>

#endif

#ifdef LINUX

/**
* LINUX CUSTOM CONFIG
*/
#define DEBUG_PACKETS // log packet debug data
#define CUSTOM_BAUD // use USB_BAUD rather than FALLBACK_BAUD (when baud rate is nonstandard)

// cross platform IO
#define XWRITE(port, data_ptr, data_len) write(port, data_ptr, data_len)
#define XREAD(port, data_ptr, data_len) read(port, data_ptr, data_len)

constexpr speed_t FALLBACK_BAUD = B115200;

#else // TEENSY

/**
* TEENSY CUSTOM CONFIG
*/
// cross platform IO
#define XWRITE(port, data_ptr, data_len) Serial.write(data_ptr, data_len)
#define XREAD(port, data_ptr, data_len) Serial.readBytes(data_ptr, data_len)

#endif

/**
* GLOBAL CONFIG
*/
#define CHECK_CRC // validate checksums


/**
* CONSTANTS
*/
constexpr int USB_BAUD = 480000000; // 480 Mbps over USB serial
constexpr int READ_BUFFER_MAX_SIZE = 1024; // max size (in bytes) of the read buffer
constexpr int WRITE_BUFFER_MAX_SIZE = 1024; // max size (in bytes) of the write buffer
constexpr int START_BYTE = 0xA5; // start byte of packet
constexpr int FULL_HEADER_SIZE = 7; // frame header + command ID
constexpr int MAX_SENSOR_ARR_SIZE = 16;
constexpr int MAX_DATA_SIZE = 1024;

const uint8_t CRC8_LOOKUP[256] = {
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

const uint16_t CRC16_LOOKUP[256] = {
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

/// @brief Simple struct to store packet data
struct SerialPacket {
    enum CommandID: uint16_t {
        DATA_REQUEST = 0x0FF,
        DR16 = 0x0103,
        REV_ENCODER = 0x0104,
        ISM = 0x0105
    };

    CommandID cmd_id;

    struct {
        CommandID cmd_id;
        uint8_t sensor_id;
    } data_req;

    struct {
        float l_stick_x, l_stick_y, r_stick_x, r_stick_y, wheel;
        uint32_t l_switch, r_switch;
    } dr16[MAX_SENSOR_ARR_SIZE];

    struct {
        float angle;
    } rev_encoder[MAX_SENSOR_ARR_SIZE];

    struct {
        float psi, theta, phi;
    } ism[MAX_SENSOR_ARR_SIZE];
};

struct RawPacket {
    SerialPacket::CommandID cmd_id;
    uint8_t data[MAX_DATA_SIZE];
    int length;

    RawPacket(SerialPacket::CommandID cmd_id): cmd_id{cmd_id} {}
};

struct SerialRequest {
    RawPacket packet;
    SerialRequest *next_req = nullptr;

    SerialRequest(RawPacket packet): packet{packet} {}
};

class SerialRequestQueue {
    private:
        SerialRequest *head;
        SerialRequest *tail;
    
    public:
        SerialRequestQueue() 
            : head{nullptr}
            , tail{nullptr}
        {}

        ~SerialRequestQueue() {
            while (head) {
                deq_request();
            }
        }

        void enq_request(RawPacket packet) {
            if (head && tail) {
                tail->next_req = new SerialRequest(packet);
                tail = tail->next_req;
            } else {
                head = tail = new SerialRequest(packet);
            }
        }

        SerialRequest *peek() {
            return tail;
        }

        void deq_request() {
            if (empty())
                return;

            SerialRequest *tmp = head;
            head = head->next_req;
            if (head == nullptr) {
                tail = nullptr;
            }

            delete tmp;
        }

        bool empty() {
            return head == nullptr && tail == nullptr;
        }
};

/// @brief Exposes basic functions for sending and receiving packets from the Teensy
class SerialComms {
    private:
        /// @brief FD of the serial port
        int port;
        
        /// @brief Sequence number which may be used to track packet loss; resets when it reaches 255
        uint8_t seq = 0;
        uint8_t prev_seq = 0;
        
        uint8_t read_buffer[READ_BUFFER_MAX_SIZE];
        uint8_t write_buffer[WRITE_BUFFER_MAX_SIZE];
        
        int read_buffer_current_size = 0;
        int write_buffer_current_size = 0;
        int read_buffer_offset = 0;
        int write_buffer_offset = 0;

        SerialRequestQueue queue;
        
        /// @brief Decodes unsigned-integer value of arbitrary size from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param size Size of the desired integer
        /// @param data Pointer to the byte array
        /// @return A 64-bit unsigned integer with the decoded value 
        uint64_t decode_uint(int offset, int size, uint8_t *data);
        
        /// @brief Decodes an unsigned short from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param data Pointer to the byte array
        /// @return An unsigned short with the decoded value
        uint16_t decode_short(int offset, uint8_t *data);

        /// @brief Decodes a float from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param data Pointer to the byte array
        /// @return A float with the decoded value
        float decode_float(int offset, uint8_t *data);

        /// @brief Decodes a double from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param data Pointer to the byte array
        /// @return A double with the decoded value
        double decode_double(int offset, uint8_t *data);

        /// @brief Read single byte from the stream
        /// @return Either -1 if no bytes are available or the byte value
        int read_byte();

        /// @brief  Read multiple bytes from the stream
        /// @param data Pointer to the data array that you want to read into
        /// @param count Desired number of bytes to be read
        /// @return The number of bytes successfully read from the stream
        int read_bytes(uint8_t *data, int count);

        int flush_write_buffer();

        /// @brief Write single byte to the write buffer
        /// @param byte Data byte
        /// @return -1 if write was unsuccessful
        int write_byte(uint8_t byte);

        /// @brief Write unsigned integer of arbitrary size to the write buffer
        /// @param u64 Data value
        /// @param size Size of the integer
        /// @return -1 if write was unsuccessful
        int write_uint(uint64_t u64, int size);

        /// @brief Write a full header to the write buffer
        /// @param length Length of the data section (check spec for details)
        void write_header(uint16_t length);

        /// @brief Write a full footer to the write buffer
        void write_footer();

        /// @brief Log error with message
        void log_err(const char* func_name, const char* msg);
        
        /// @brief Log error without message
        void log_err(const char* func_name);

        int calc_data_response_size(SerialPacket::CommandID cmd_id);
        uint8_t generate_crc8(const uint8_t *data, int size);
        uint16_t generate_crc16(const uint8_t *data, int size);
        
    public:
        SerialComms(const char *port_name);
        ~SerialComms();

        /// @brief Send packet to the Teensy
        /// @param cmd_id Command ID of the packet
        /// @param data Pointer to the byte array with packet data
        /// @param length Length of the data section
        void send_packet(const RawPacket &packet);
        
        /// @brief Attempt to read packet from 
        /// @return -1 if unable to detect packet in the stream
        bool read_packet(SerialPacket& packet);

        /// @brief enqueue a packet and send
        void request_data(SerialPacket::CommandID cmd_id, uint8_t sensor_id);

        /// @brief enqueue a packet (add SerialRequest to the queue)
        void enqueue_packet(SerialPacket::CommandID cmd_id, uint8_t *data, uint16_t length);

        /// @brief enqueue data request (KHADAS)
        void enqueue_data_request(SerialPacket::CommandID cmd_id, uint8_t sensor_id);

        /// @brief send as many packets as possible on this loop iteration
        int flush_queue(int loop_freq);
};

#endif