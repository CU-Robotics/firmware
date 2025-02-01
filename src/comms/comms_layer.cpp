#include "comms_layer.hpp"

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
    // get ethernet incoming/outgoing packet references
    EthernetPacket *eth_outgoing_ptr = Ethernet.get_outgoing_packet();
    EthernetPacket *eth_incoming_ptr = Ethernet.get_incoming_packet();


    // give Ethernet loop outgoing data,
    *eth_outgoing_ptr = encode(data_outgoing_ethernet, PhysicalMedium::Ethernet);
    
    // run Ethernet loop,
    Ethernet.loop();

    // and get incoming data from Ethernet loop
    data_incoming_ethernet = decode(*eth_incoming_ptr, PhysicalMedium::Ethernet);

    return 0;
}

void CommsLayer::send(CommsData&& data, PhysicalMedium medium) {
    send(data, medium);
}

void CommsLayer::send(CommsData& data, PhysicalMedium medium) {
    // TODO: FirmwareData is not an atomic data structure, 
    // but we treat it as such for now

    data_outgoing_ethernet = &data;
}

HiveData CommsLayer::receive(PhysicalMedium medium) {
    // TODO: HiveData needs to pull blocks out of the queue and reconstruct them,
    // but for now we treat HiveData as an atomic data structure

    return data_incoming_ethernet;
}



//
// CommsLayer PRIVATE definitions
//


EthernetPacket CommsLayer::encode(CommsData *source_data, PhysicalMedium medium) {
    // encode the CommsData found at data_outgoing_ethernet
    return EthernetPacket();
}

HiveData CommsLayer::decode(EthernetPacket source_packet, PhysicalMedium medium) {
    // decode the EthernetPacket found at data_incoming_ethernet
    return HiveData();
}




} // namespace Comms    