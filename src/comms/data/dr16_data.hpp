#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

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
            bool w     : 1;
            bool s     : 1;
            bool a     : 1;
            bool d     : 1;
            bool shift : 1;
            bool ctrl  : 1;
            bool q     : 1;
            bool e     : 1;
            bool r     : 1;
            bool f     : 1;
            bool g     : 1;
            bool z     : 1;
            bool x     : 1;
            bool c     : 1;
            bool v     : 1;
            bool b     : 1;
        } key;
    } keys;
};
