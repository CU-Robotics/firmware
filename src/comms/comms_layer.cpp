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
    if(data_outgoing_ethernet != nullptr){
        *eth_outgoing_ptr = encode(data_outgoing_ethernet);
    }

    // run Ethernet loop,
    Ethernet.loop();

    // and get incoming data from Ethernet loop
    data_incoming_ethernet = decode(*eth_incoming_ptr);
    
    return 0;
}

void CommsLayer::send(CommsData&& data, PhysicalMedium medium) {
    // passthrough with rvalue to main send
    send(data, medium);
}

void CommsLayer::send(CommsData& data, PhysicalMedium medium) {
    data_outgoing_ethernet = &data;
}

HiveData CommsLayer::receive(PhysicalMedium medium) {
    return data_incoming_ethernet;
}



//
// CommsLayer PRIVATE definitions
//


EthernetPacket CommsLayer::encode(CommsData *source_data) {
    
    // encode the CommsData found at data_outgoing_ethernet
    EthernetPacket retval;

    // we don't really care what kind of data source_data is,
    // we just need its size
    retval.header.payload_size = source_data->size;
    if(source_data->priority == Priority::High) retval.header.flags = EthernetPacketType::PRIORITY; // set priority if appropriate
    memcpy(retval.payload.data, source_data, source_data->size);

    return retval;
}

HiveData CommsLayer::decode(EthernetPacket source_packet) {
    // decode the EthernetPacket found at data_incoming_ethernet
    HiveData retval;
    
    // construct a CommsData object to test for source_packet payload properties
    CommsData test_data;
    memcpy(&test_data, source_packet.payload.data, sizeof(CommsData));

    // figure out what we just pulled from source_packet
    if(test_data.type_label == TypeLabel::ExampleHive){
        memcpy(&retval.hive_str, source_packet.payload.data, sizeof(HiveString));
    }

    return retval;
}




} // namespace Comms    