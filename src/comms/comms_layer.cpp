#include "comms_layer.hpp"

#include "config_layer.hpp" // ONLY used for sd config storage in teensy_configure(). will be removed eventually

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

    Serial.printf("DEBUG: sizeof configdata: %d\n", sizeof(ConfigData));    // delete me... please...... when you read it for the first time..... please.....

    return 0;
}

int CommsLayer::loop() {
    Ethernet.loop();

    EthernetPacket* p_incoming = Ethernet.get_incoming_packet();   
    // EthernetPacket* p_outgoing = Ethernet.get_outgoing_packet();
    
    // check for config packet from ethernet
    if(p_incoming->header.flags == Comms::EthernetPacketFlags::CONFIG) {
        // reconfigure and trigger reboot
        teensy_configure(*p_incoming);
        while (1) ;     // should NEVER start
    }

    return 0;
}



// - data I/O functions

// take data, convert it into ethernet compatible form (packet sequence)
EthernetPacket CommsLayer::encode(FirmwareData data, int data_type, int data_flag) {
    // TODO

    return EthernetPacket();
}

// take ethernet payload, convert it into TeensyData
HiveData CommsLayer::decode(EthernetPacket packet) {
    // TODO
    return HiveData();
}

// transmit a given EthernetPacket
int CommsLayer::transmit(EthernetPacket packet) {
    // TODO

    return 0;
}

// receive an EthernetPacket
// nullptr if failed, else success
EthernetPacket* CommsLayer::receive() {
    // TODO
    return nullptr;
}


// - config

int CommsLayer::teensy_configure(EthernetPacket &config_packet) {
    // decode from packet

    // send ack for config packet

    // write the outgoing packet
    
    // store config on SD card
    // NOTE: currently SD config is handled by config_layer.cpp. this will be phased out
    // when we replace HID comms and integrate with comms_layer. so, this is CURRENTLY 
    // going to dump our config_data into a config_layer config object and store that
    // for config_layer.cpp to handle on reboot. this is GOING to have to be replaced eventually


    return 0;
}



//
// CommsLayer PRIVATE definitions
//

EthernetPacket CommsLayer::construct_packet(uint8_t* bytes, uint16_t payload_size, uint8_t type, uint8_t flag, uint64_t timestamp) {
    EthernetPacket output;

    // copy over bytes into output.payload, keep separate from byte buffer (bytes)
    memcpy(output.payload.data, bytes, payload_size);

    // set header attributes
    output.header.time_stamp = timestamp;
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