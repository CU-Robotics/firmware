#ifndef USB_HID_HPP
#define USB_HID_HPP

#include "Arduino.h"

#define PACKET_SIZE_BYTES 1024

union float_byte {
    float number;
    byte bytes[4];
};

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
        int next_free();

    private:
        byte packet [PACKET_SIZE_BYTES];
        int next_free_index = 0;
};


#endif