#include "usb_hid.hpp"

usbHID::usbHID(){
    clear();
    put_float(0, -1);
}

void usbHID::clear(){
    for(int i = 0; i < PACKET_SIZE_BYTES; i++) {
        packet[i] = 0.0;
    }
    next_free_index = 0;
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
    
    next_free_index = index+4;
}

float usbHID::get_float(int index){
    float_byte fb;

    fb.bytes[0] = packet[index];
    fb.bytes[1] = packet[index+1];
    fb.bytes[2] = packet[index+2];
    fb.bytes[3] = packet[index+3];

    return fb.number;
}

int usbHID::next_free(){
    return (next_free_index > PACKET_SIZE_BYTES ? (int) PACKET_SIZE_BYTES : next_free_index);
}