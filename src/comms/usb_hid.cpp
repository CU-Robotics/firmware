#include "usb_hid.hpp"


void usbHid::usbHid(){
    clear();
    packet_count = 0;
    put_float(0, -1);
}

void usbHID::clear(){
    for(int i = 0; i < PACKET_SIZE_BYTES; i++) {
        packet[i] = 0.0;
    }
}

int8_t usbHID::read(){
   return usb_rawhid_recv(packet, 0);
}

int8_t usbHID::write(){
    return usb_rawhid_send(packet, 0);
}

void usbHID::put_float(int index, float f){
    float_byte fb;
    fb.number = f;
    
    packet[index] = fb.bytes[0];
    packet[index+1] = fb.bytes[1];
    packet[index+2] = fb.bytes[2];
    packet[index+3] = fb.bytes[3];
}

float usbHID::get_float(int index){
    float_byte fb;

    fb.bytes[0] = packet[index];
    fb.bytes[1] = packet[index+1];
    fb.bytes[2] = packet[index+2];
    fb.bytes[3] = packet[index+3]

    return fb.number;
}