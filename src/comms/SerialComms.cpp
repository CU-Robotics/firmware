#include "SerialComms.hpp"

SerialComms::SerialComms(const char *port_name) 
{
    queue = SerialRequestQueue();

#ifdef LINUX
    // attempt to open the serial port
    port = open(port_name, O_RDWR);

    // check if Teensy is connected
    if (port < 0) {
        log_err("open", "Teensy may be disconnected");
    } else {
        // Configure the serial port settings
        struct termios tty;
        struct serial_struct serial;

        if (tcgetattr(port, &tty) < 0) {
            log_err("tcgetattr");
        }

    #ifdef CUSTOM_BAUD

    #ifdef DEBUG_PACKETS
        fprintf(stderr, "configuring custom baud (%.2fMbps)\n", USB_BAUD / 1.0e+6);
    #endif

        // on linux custom baud must be set through baud rate aliasing
        serial.reserved_char[0] = 0;
        
        if (ioctl(port, TIOCGSERIAL, &serial) < 0) {
            log_err("ioctl[TIOCGSERIAL]");
        }

        serial.flags &= ~ASYNC_SPD_MASK;
        serial.flags |= ASYNC_SPD_CUST;
        serial.custom_divisor = (serial.baud_base + (USB_BAUD / 2)) / USB_BAUD;
        
        if (serial.custom_divisor < 1) {
            serial.custom_divisor = 1;
        }

        if (ioctl(port, TIOCSSERIAL, &serial) < 0) {
            log_err("ioctl[TIOCSSERIAL]");
        }

        if (ioctl(port, TIOCGSERIAL, &serial) < 0) {
            log_err("iotcl[TIOCSSERIAL]");
        }

        if (serial.custom_divisor * USB_BAUD != serial.baud_base) {
            warnx("actual baudrate is %d / %d = %f",
                  serial.baud_base, serial.custom_divisor,
                  (float) serial.baud_base / serial.custom_divisor);
        }

        if (cfsetospeed(&tty, B38400) < 0) {
            log_err("cfsetospeed");
        }

        if (cfsetispeed(&tty, B38400) < 0) {
            log_err("cfsetispeed");
        }

    #else // B<rate>

        // if baud rate is one of the B<rate> constants
        if (cfsetospeed(&tty, FALLBACK_BAUD) < 0) {
            log_err("cfsetospeed");
        }

        if (cfsetispeed(&tty, FALLBACK_BAUD) < 0) {
            log_err("cfsetispeed");
        }
        
    #endif

        // setting "raw mode" attributes according to man7.org/linux/man-pages/man3/termios3.html
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;    // No parity
        tty.c_cflag &= ~CSTOPB;    // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;        // 8 data bits
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        
        // set VMIN and VTIME to 0 (for total non-blocking reads)
        // man page warns against this so alternatives are welcomed
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        
        if (tcsetattr(port, TCSANOW, &tty) < 0) {
            log_err("tcsetattr");
        }
    }
#else // TEENSY
    // baud rate setting is ignored, native 480 Mbps always used
    port = 0;
    Serial.begin(USB_BAUD);
#endif
}

SerialComms::~SerialComms() 
{
#ifdef LINUX
    // attempt to close the serial port
    if (close(this->port) < 0) {
        log_err("close", "Teensy may be disconnected");
    }
#endif // no handles to release on teensy
}

void SerialComms::log_err(const char* func_name, const char* msg)
{
#if defined(LINUX) && defined(DEBUG_PACKETS)
    fprintf(stderr, "\"%s\" call failed (%s): errno = %d\n", func_name, msg, errno);
#endif // no stdout on teensy
}

void SerialComms::log_err(const char* func_name)
{
#if defined(LINUX) && defined(DEBUG_PACKETS)
    fprintf(stderr, "\"%s\" call failed: errno = %d\n", func_name, errno);
#endif // no stdout on teensy
}

uint8_t SerialComms::generate_crc8(const uint8_t *data, int size)
{
    uint8_t crc8 = 0xFF;
    for (int i = 0; i < size; i++) {
        uint8_t n = crc8 ^ data[i];
        crc8 = CRC8_LOOKUP[n];
    }
    return crc8;
}

uint16_t SerialComms::generate_crc16(const uint8_t *data, int size)
{
    uint16_t crc16 = 0xFFFF;
    for (int i = 0; i < size; i++) {
        uint8_t n = data[i];
        crc16 = (crc16 >> 8) ^ CRC16_LOOKUP[(crc16 ^ static_cast<uint16_t>(n)) & 0x00FF];
    }
    return crc16;
}

int SerialComms::flush_write_buffer()
{
    if (XWRITE(port, write_buffer, write_buffer_offset) < 0) {
        log_err("write");
        return -1;
    }
    write_buffer_offset = 0;
    return 0;
}

int SerialComms::write_byte(uint8_t byte)
{
    // check to make sure we don't write off the end of the buffer
    if (write_buffer_offset >= WRITE_BUFFER_MAX_SIZE) {
        if (flush_write_buffer() < 0) {
            log_err("flush_write_buffer");
            return -1;
        }
    }
    
    write_buffer[write_buffer_offset++] = byte;
    return 0;
}

int SerialComms::write_uint(uint64_t u64, int size)
{
    // if requesting 1 byte, just stage it directly
    if (size == 1) {
        return write_byte((uint8_t) u64);
    } else {
        // serialize integer into respective bytes in little endian order
        for (int i = 0; i < size; i++) {
            uint8_t byte = (u64 >> i * 8) & 0xFF;
            if (write_byte(byte) < 0) {
                return -1;
            }
        }
        return 0;
    }
}

void SerialComms::write_header(uint16_t length)
{
    // stage all components of header
    write_byte(START_BYTE);
    write_uint(length, 2);
    write_byte(seq);

    // update packet sequence number if must wrap around
    if (seq == 255) {
        seq = 0;
    } else {
        seq++;
    }

    // TODO: implement crc8
    write_byte(generate_crc8(write_buffer, write_buffer_offset));
}

void SerialComms::write_footer()
{
    uint16_t crc16 = generate_crc16(write_buffer, write_buffer_offset);
    write_uint(crc16, 2);
}

void SerialComms::send_packet(const RawPacket &packet)
{
    // stage all components of packet
    write_header(packet.length);
    write_uint(static_cast<uint16_t>(packet.cmd_id), 2);

    for (int i = 0; i < packet.length; i++) {
        write_byte(packet.data[i]);
    }

    write_footer();
    flush_write_buffer();
}

int SerialComms::read_byte()
{
#ifdef LINUX
    // if we are at the end of the read buffer, overwrite from beginning
    if (read_buffer_offset == read_buffer_current_size) {
        int result = read(port, read_buffer, READ_BUFFER_MAX_SIZE);

        if (result >= 0) {
            read_buffer_current_size = result; 
            read_buffer_offset = 0;
        } else {
            log_err("read");
            read_buffer_current_size = 0;
            read_buffer_offset = 0;
        }
    }

    if (read_buffer_current_size > 0) {
        return (int) read_buffer[read_buffer_offset++];
    } else {
        return -1;
    }
#else // TEENSY
    return Serial.read();
#endif
}

int SerialComms::read_bytes(uint8_t *data, int count)
{
#ifdef LINUX
    int read_count;
    for (read_count = 0; read_count < count; read_count++) {
        int byte = read_byte();
        if (byte >= 0) {
            data[read_count] = static_cast<uint8_t>(byte);
        } else {
            break;
        }
    }
    return read_count;
#else
    return Serial.readBytes(reinterpret_cast<char*>(data), count);
#endif
}

uint64_t SerialComms::decode_uint(int offset, int size, uint8_t *data)
{
    // if we just want 1 byte, pull from array directly
    if (size == 1) {
        return data[offset];
    } else {
        // must assemble integer from bytes in little endian order
        uint64_t result = 0;
        for (int i = 0; i < size; i++) {
            uint8_t byte = data[offset + i];
            result = result | (byte << i * 8);
        }
        return result;
    }
}

uint16_t SerialComms::decode_short(int offset, uint8_t *data)
{
    return static_cast<uint16_t>(decode_uint(offset, 2, data));
}

int SerialComms::calc_data_response_size(SerialPacket::CommandID cmd_id)
{
    switch (cmd_id) {
        case SerialPacket::DR16:
            return 29;
        case SerialPacket::REV_ENCODER:
            return 5;
        case SerialPacket::ISM:
            return 13;
        default:
            return 0;
    }
    return 0;
}

void SerialComms::enqueue_packet(SerialPacket::CommandID cmd_id, uint8_t *data, uint16_t length)
{
    RawPacket packet = RawPacket(cmd_id);

    for (int i = 0; i < length; i++) {
        packet.data[i] = data[i];
    }

    packet.length = length;
    queue.enq_request(packet);
}

void SerialComms::enqueue_data_request(SerialPacket::CommandID cmd_id, uint8_t sensor_id)
{
#ifdef LINUX
    uint8_t data[3];
    data[0] = cmd_id & 0xFF;
    data[1] = cmd_id >> 8;
    data[2] = sensor_id;
    enqueue_packet(SerialPacket::DATA_REQUEST, data, 3);
#endif
}

/// @brief send as many packets as possible on this loop iteration
int SerialComms::flush_queue(int loop_freq)
{
    int bitrate_limit = USB_BAUD / loop_freq;
    int throughput = 0;

    while (!queue.empty() && bitrate_limit >= 0) {
        SerialRequest *curr_req = queue.peek();
        send_packet(curr_req->packet);
        bitrate_limit -= calc_data_response_size(curr_req->packet.cmd_id);
        queue.deq_request();
        throughput++;
    }

    return throughput;
}

bool SerialComms::read_packet(SerialPacket& packet)
{
    int byte;
    uint8_t temp[4];

    // try to pull bytes from the stream until a start byte is encountered
    while ((byte = read_byte()) >= 0) {
        if (byte == START_BYTE) {

        #if defined(DEBUG_PACKETS) && defined(LINUX)
            fprintf(stderr, "\n[ATTEMPTING PACKET READ]\n");
        #endif

            // read the packet header
            if (read_bytes(temp, 4) < 4) {
                break;
            }

            uint16_t data_length = decode_short(0, temp);
            uint8_t seq = temp[2];
            uint8_t crc8 = temp[3];

            // check for dropped packet
            if (seq - prev_seq > 1) {
            #if defined(DEBUG_PACKETS) && defined(LINUX)
                fprintf(stderr, "\t(!) packet dropped\n");
            #endif
            }
            prev_seq = seq;

            // read the packet data section
            if (read_bytes(temp, 2) < 2) {
                break;
            }

            uint16_t cmd_id = decode_short(0, temp);
            uint8_t packet_bytes[FULL_HEADER_SIZE + data_length];
            
            packet_bytes[0] = START_BYTE;
            packet_bytes[1] = data_length & 0xFF;
            packet_bytes[2] = data_length >> 8;
            packet_bytes[3] = seq;
            packet_bytes[4] = crc8;
            packet_bytes[5] = cmd_id & 0xFF;
            packet_bytes[6] = cmd_id >> 8;

            // data segment starts after the header/command ID
            uint8_t *data = &packet_bytes[FULL_HEADER_SIZE];
            uint8_t calc_crc8 = generate_crc8(packet_bytes, 4);

        #if defined(DEBUG_PACKETS) && defined(LINUX)
            fprintf(stderr, "\tpacket sequence num = %u\n", seq);
            fprintf(stderr, "\tdata_length = %u\n", data_length);
            fprintf(stderr, "\treceived crc8 = %u\n", crc8);
            fprintf(stderr, "\tcalculated crc8 = %u\n", calc_crc8);
        #endif

        #ifdef CHECK_CRC
            if (crc8 != calc_crc8) {
            #if defined(DEBUG_PACKETS) && defined(LINUX)
                fprintf(stderr, "\t(!) CRC8 checksums do not match\n");
            #endif

                break;
            }
        #endif
        
            if (read_bytes(data, data_length) < data_length) {
                break;
            }

            // read the packet footer
            if (read_bytes(temp, 2) < 2) {
                break;
            }

            uint16_t crc16 = decode_short(0, temp);
            uint16_t calc_crc16 = generate_crc16(packet_bytes, FULL_HEADER_SIZE + data_length);
            
        #if defined(DEBUG_PACKETS) && defined(LINUX)
            fprintf(stderr, "\treceived crc16 = %u\n", crc16);
            fprintf(stderr, "\tcalculated crc16 = %u\n", calc_crc16);
        #endif
            
        #ifdef CHECK_CRC
            if (crc16 != calc_crc16) {          
            #if defined(DEBUG_PACKETS) && defined(LINUX)
                fprintf(stderr, "\t(!) CRC16 checksums do not match\n");
            #endif
                
                break;
            }
        #endif

            // utility arrays for decoding packet data
            float float_arr[16] = {0.0};
            uint32_t uint32_arr[16] {0};
            uint8_t sensor_id;

            union {
                uint32_t u32;
                float f32;
            } bitcaster;

            packet.cmd_id = static_cast<SerialPacket::CommandID>(cmd_id);
            switch (cmd_id) {
                /**
                * TEENSY SIDE
                */
                case SerialPacket::DATA_REQUEST:
                    packet.data_req.cmd_id = static_cast<SerialPacket::CommandID>(decode_uint(0, 2, data));
                    packet.data_req.sensor_id = data[3];
                    break;

                /**
                * KHADAS SIDE
                */
                // sensors and estimators
                case 0x0101: // estimator state
                    break;
                case 0x0102: // motor feedback
                    break;
                case SerialPacket::DR16: // DR16 data
                    sensor_id = data[0];

                    for (int i = 0; i < 5; i++) {
                        bitcaster.u32 = static_cast<uint32_t>(decode_uint(1 + i * 4, 4, data));
                        float_arr[i] = bitcaster.f32;
                    }

                    for (int i = 0; i < 2; i++) {
                        uint32_arr[i] = static_cast<uint32_t>(decode_uint(1 + 20 + i * 4, 4, data));
                    }

                    packet.dr16[sensor_id].l_stick_x = float_arr[0];
                    packet.dr16[sensor_id].l_stick_y = float_arr[1];
                    packet.dr16[sensor_id].r_stick_x = float_arr[2];
                    packet.dr16[sensor_id].r_stick_y = float_arr[3];
                    packet.dr16[sensor_id].wheel = float_arr[4];
                    packet.dr16[sensor_id].l_switch = uint32_arr[0];
                    packet.dr16[sensor_id].r_switch = uint32_arr[1];

                #if defined(DEBUG_PACKETS) && defined(LINUX)
                    fprintf(
                        stderr,
                        "\tdr16 packet data (%#02x):\n"
                        "\t\tl_stick_x: %f\n"
                        "\t\tl_stick_y: %f\n"
                        "\t\tr_stick_x: %f\n"
                        "\t\tr_stick_y: %f\n"
                        "\t\twheel: %f\n"
                        "\t\tl_switch: %u\n"
                        "\t\tr_switch: %u\n",
                        packet.cmd_id,
                        packet.dr16[sensor_id].l_stick_x,
                        packet.dr16[sensor_id].l_stick_y,
                        packet.dr16[sensor_id].r_stick_x,
                        packet.dr16[sensor_id].r_stick_y,
                        packet.dr16[sensor_id].wheel,
                        packet.dr16[sensor_id].l_switch,
                        packet.dr16[sensor_id].r_switch
                    );
                #endif
                    
                    break;
                case SerialPacket::REV_ENCODER: // rev encoder
                    sensor_id = data[0];
                    bitcaster.u32 = static_cast<uint32_t>(decode_uint(1, 4, data));
                    packet.rev_encoder[sensor_id].angle = bitcaster.f32;

                #if defined(DEBUG_PACKETS) && defined(LINUX)
                    fprintf(
                        stderr,
                        "\tRev Encoder packet data (%#02x):\n"
                        "\t\tangle: %f\n",
                        packet.cmd_id,
                        packet.rev_encoder[sensor_id].angle * 180/3.14159
                    );
                #endif

                    break;
                case SerialPacket::ISM: // ISM
                    sensor_id = data[0];

                    for (int i = 0; i < 3; i++) {
                        bitcaster.u32 = static_cast<uint32_t>(decode_uint(1 + i * 4, 4, data));
                        float_arr[i] = bitcaster.f32;
                    }

                    packet.ism[sensor_id].psi = float_arr[0];
                    packet.ism[sensor_id].theta = float_arr[1];
                    packet.ism[sensor_id].phi = float_arr[2];

                #if defined(DEBUG_PACKETS) && defined(LINUX)
                    fprintf(
                        stderr,
                        "\tISM packet data (%#02x):\n"
                        "\t\tpsi: %f\n"
                        "\t\tpsi: %f\n"
                        "\t\tpsi: %f\n",
                        packet.cmd_id,
                        packet.ism[sensor_id].psi,
                        packet.ism[sensor_id].theta,
                        packet.ism[sensor_id].phi
                    );
                #endif

                    break;

                // referee system
                case 0x0201: // referee system data
                    break;
                case 0x0202: // client draw command
                    break;
            }

            // successful packet read
            return true;
        }
    }
    
    // while loop broke, packet was not found
    return false;
}