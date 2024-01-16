#include "usb_hid.hpp"

int8_t usbHID::read(){
   return usb_rawhid_recv(packet, 0);
}

int8_t usbHID::write(){
    return usb_rawhid_send(packet, 0);
}

byte USBHID::get(int index){
    return packet[index];
}

void USBHID::put(int index, byte* value){
    packet[index] = value;
}
