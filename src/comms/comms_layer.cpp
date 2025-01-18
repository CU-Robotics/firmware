#include "comms_layer.hpp"

#include "config_layer.hpp" 

extern "C" void reset_teensy(void);

namespace Comms {

// EthernetPackage defs

EthernetPackage::EthernetPackage() {
// set all packets to default ethernet packet, invalid to indicate that it does not need to be read
for (int i = 0; i < MAX_ETHERNET_PACKETS; i++) {
    packets[i] = EthernetPacket();
    packets[i].header.flags = EthernetPacketFlags::INVALID;
    packets[i].header.type = EthernetPacketType::DEBUG;
}
flag = EthernetPacketFlags::INVALID;
num_packets = 0;
}

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



// - data I/O functions

// take data, convert it into ethernet compatible form (packet sequence)
EthernetPackage CommsLayer::encode(FirmwareData data, int data_type, int data_flag) {
    // TODO

    return EthernetPackage();
}

// take ethernet payload, convert it into TeensyData
HiveData CommsLayer::decode(EthernetPackage packet) {
    // TODO
    return HiveData();
}

// transmit a given EthernetPacket
int CommsLayer::transmit(EthernetPackage packet) {
    // TODO

    return 0;
}

// receive an EthernetPacket
// nullptr if failed, else success
EthernetPackage CommsLayer::receive() {
    // TODO
    return EthernetPackage();
}


// - config

int CommsLayer::teensy_configure() {
    // TODO
    
    return 0;    
}



//
// CommsLayer PRIVATE definitions
//

EthernetPacket CommsLayer::construct_packet(uint8_t* bytes, uint16_t payload_size, uint8_t type, uint8_t flag) {
    EthernetPacket output;

    // copy over bytes into output.payload, keep separate from byte buffer (bytes)
    if(payload_size > 0) memcpy(output.payload.data, bytes, payload_size);

    // set header attributes
    output.header.sequence = sequence++;    // increment to count num of packets created for transmission (treated as ID)
    output.header.payload_size = payload_size;
    output.header.type = type;
    output.header.flags = flag;

    return output;
}

EthernetPacket CommsLayer::construct_EOT_packet() {
    // used later for ethernet payload transmissions, currently not needed

    EthernetPacket eot;
    eot.header.flags = NORMAL;
    eot.header.type = EOT;

    return eot;
}

} // namespace Comms    