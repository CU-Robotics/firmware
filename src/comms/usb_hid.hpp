#ifndef USB_HID_HPP
#define USB_HID_HPP

#include "Arduino.h"

#define PACKET_SIZE_BYTES 1024

def union{
    float number;
    byte bytes[4]
} float_byte;

class usbHID {
    public:
        usbHID();
        void clear();

        int8_t write();
        int8_t read();

        void put(int index, byte value);
        byte get(int index);

        float get_float(int index);
        void put_float(int index, float f);

    private:
        byte packet [PACKET_SIZE_BYTES];
        int packet_count;
};


#endif