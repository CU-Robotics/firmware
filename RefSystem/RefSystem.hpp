#ifndef REF_SYSTEM_HPP
#define REF_SYSTEM_HPP

#include <cassert>
#include "Arduino.h"

#include "RefSystemPacketDefs.hpp"


class RefSystem
{
public:
    RefSystem();

    void init();

    void read(uint16_t filterID = 0x0);
    void write(Frame& frame);

public:
    void pretty_print(Frame& frame);

private:
    bool read_frame_header(Frame& frame);
    bool read_frame_command_ID(Frame& frame);
    bool read_frame_data(Frame& frame);
    bool read_frame_CRC(Frame& frame);

private:
    uint8_t raw_buffer[REF_MAX_PACKET_SIZE] = { 0 };

};

#endif // REF_SYSTEM_HPP