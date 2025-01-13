#include "comms_layer.hpp"


namespace Comms {

//
// EthernetPayload definitions
//

EthernetPayload::EthernetPayload() {
    packets = nullptr;      // do not point to anything initially
    length = 0;             // len==0 implies that packets is not readable
}

EthernetPayload::~EthernetPayload() {
    // want to free all elements of packets
    // assume length of packets == length. be careful and make sure this value is right!!
    for (unsigned int i = 0; i < length; i++) {
        delete& packets[i];
    }
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

    EthernetPacket p_incoming = ethernet_packet_receive();      // funny function to only read new packets
    EthernetPacket* p_outgoing = Ethernet.get_outgoing_packet();
    
    // handle config packet
    if(p_incoming.header.flags == Comms::EthernetPacketFlags::CONFIG) {
        Serial.printf("Config packet received, id = %d, ts = %d\n", p_incoming.header.sequence, p_incoming.header.time_stamp);
        
        uint8_t output_buffer[ETHERNET_PACKET_MAX_SIZE];
        memset(output_buffer, 0, 4096);
        // set outgoing packet to be data ack
        *p_outgoing = construct_packet(
            output_buffer, 
            0, 
            Comms::EthernetPacketType::DATA, 
            Comms::EthernetPacketFlags::ACK, 
            millis()
        );
    }

    return 0;
}



// - data I/O functions

// take data, convert it into ethernet compatible form (packet sequence)
EthernetPayload CommsLayer::encode(FirmwareData data, int data_type, int data_flag) {
    // TODO

    // DEBUG: testing EthernetPayload transmission, just need one packet for now
    EthernetPayload sample;
    sample.packets = new EthernetPacket;
    sample.packets[0] = construct_packet((uint8_t*)(&data), sizeof(FirmwareData), data_type, data_flag, 0);
    sample.type = data_type;
    sample.flag = data_flag;
    // sample.section_sizes[0] = sizeof(HiveData);

    return sample;
}

// take ethernet payload, convert it into TeensyData
HiveData CommsLayer::decode(EthernetPayload payload) {
    // TODO
    return HiveData();
}

// transmit a given EthernetPayload
int CommsLayer::transmit(EthernetPayload* payload) {
    // TODO

    return 0;
}

// receive an EthernetPayload
// nullptr if failed, else success
EthernetPayload* CommsLayer::receive() {
    // TODO
    return nullptr;
}


// - config

int CommsLayer::teensy_configure() {
    // TODO
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

EthernetPacket CommsLayer::ethernet_packet_receive() {
    // copy current contents of packet buffer into new packet)
    EthernetPacket incoming;
    EthernetPacket* incoming_buffer = Ethernet.get_incoming_packet();
    memcpy(&incoming, incoming_buffer, sizeof(EthernetPacket));
    if(ethernet_is_valid_packet(incoming)) {
        // packet is new
        // write empty (invalid) packet to incoming buffer, and return the incoming new packet
        // see note in ethernet_is_valid_packet on invalid packet construction
        uint8_t null_buffer[ETHERNET_PACKET_MAX_SIZE];
        memset(null_buffer, 0, ETHERNET_PACKET_MAX_SIZE);
        *incoming_buffer = construct_packet(
            null_buffer, 
            0, 
            Comms::EthernetPacketType::DEBUG,
            Comms::EthernetPacketFlags::INVALID,
            0
        );

        return incoming;
    }
    // invalid packet detected
    uint8_t null_buffer[ETHERNET_PACKET_MAX_SIZE];
    memset(null_buffer, 0, ETHERNET_PACKET_MAX_SIZE);
    EthernetPacket null_packet = construct_packet(
        null_buffer, 
        0, 
        Comms::EthernetPacketType::DEBUG,
        Comms::EthernetPacketFlags::INVALID,
        0
    );
    return null_packet;
}

bool CommsLayer::ethernet_is_valid_packet(EthernetPacket packet) {
    // when we need to "delete" a packet in a buffer, we will simply write an empty 
    // packet with INVALID flag. this function will detect if a packet is therefore "deleted" (invalid)


    return (packet.header.flags == Comms::EthernetPacketFlags::INVALID);
}

} // namespace Comms    