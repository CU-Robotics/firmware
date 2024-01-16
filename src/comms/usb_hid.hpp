#ifndef USB_HID_HPP
#define USB_HID_HPP

#include "Arduino.h"

#define PACKET_SIZE_BYTES 1024


class usbHID{

    public:
        usbHID();

        int8_t write();
        int8_t read();

        void put(int index, byte value);
        byte get(int index);

    private:
        byte packet [PACKET_SIZE_BYTES];
};


#endif