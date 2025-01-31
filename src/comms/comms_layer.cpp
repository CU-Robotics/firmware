#include "comms_layer.hpp"

#include "config_layer.hpp" 

namespace Comms {


//
// CommsLayer PUBLIC definitions
//

// - init functions

int CommsLayer::init() {
    sequence = 0;
    bool success = Ethernet.begin();
    if (success) {
        Serial.printf("Ethernet socket online\n");
    } else {
        Serial.printf("Ethernet FAILED to initialize, exiting...\n");
        return -1;
    }

    return 0;
}

int CommsLayer::loop() {
    Ethernet.loop();

    // EthernetPacket* p_incoming = Ethernet.get_incoming_packet();   
    // EthernetPacket* p_outgoing = Ethernet.get_outgoing_packet();
    
    return 0;
}

void CommsLayer::send(CommsData* data, PhysicalMedium medium) {
    switch(medium) {
    case PhysicalMedium::Ethernet:
        data_outgoing_ethernet = data;
        break;
    case PhysicalMedium::HID:
        data_outgoing_HID = data;
        break;
    }
    return;
}

HiveData CommsLayer::receive(PhysicalMedium medium) {


    return HiveData();
}



//
// CommsLayer PRIVATE definitions
//



} // namespace Comms    