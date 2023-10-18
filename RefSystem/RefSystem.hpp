#ifndef REF_SYSTEM_HPP
#define REF_SYSTEM_HPP

#include <cassert>
#include "Arduino.h"

#include "RefSystemPacketDefs.hpp"


class RefSystem
{
public:
    RefSystem();

    void Init();

    bool Read();

    bool Process();

public:


public:
    bool ReadFrameStart(Frame& frame);
    bool ReadFrameCommandID(Frame& frame);
    bool ReadFrameData(Frame& frame);
    bool ReadFrameEnd(Frame& frame);

private:
    uint8_t m_rawData[REF_MAX_FRAME_DATA_SIZE] = { 0 };
    uint8_t m_frameDataSection[REF_MAX_FRAME_DATA_SIZE] = { 0 };

};

#endif // REF_SYSTEM_HPP