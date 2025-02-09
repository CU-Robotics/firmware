#include "comms_layer.hpp"

namespace Comms {


//
// CommsLayer PUBLIC definitions
//

// - init functions

int CommsLayer::init() {
    return init(true, true);
}

int CommsLayer::init(bool e_Ethernet, bool e_HID __attribute__((unused))) {
    sequence = 0;
    if(e_Ethernet){
        bool success = Ethernet.begin();
        if (success) {
            Serial.printf("Ethernet socket online\n");
        } else {
            Serial.printf("Ethernet FAILED to initialize, exiting...\n");
            return -1;
        }
        enable_Ethernet = e_Ethernet;
    }
    

    return 0;
}

int CommsLayer::loop() {
    if(enable_Ethernet) {

        // get ethernet incoming/outgoing packet references
        EthernetPacket *eth_outgoing_ptr = Ethernet.get_outgoing_packet();
        EthernetPacket *eth_incoming_ptr = Ethernet.get_incoming_packet();

        // give Ethernet loop outgoing data,
        EthernetPacket outgoing = encode();
        *eth_outgoing_ptr = outgoing;

        // run Ethernet loop,
        Ethernet.loop();

        // and get incoming data from Ethernet loop
        EthernetPacket incoming = *eth_incoming_ptr;
        decode(incoming);
    }

    return 0;
}

void CommsLayer::send(CommsData* data, PhysicalMedium medium) {

}

HiveData CommsLayer::receive(PhysicalMedium medium) {
    return data_incoming_ethernet;
}



//
// CommsLayer PRIVATE definitions
//


EthernetPacket CommsLayer::encode() {
    // encode from PacketPayload data_outgoing_ethernet, packetpayload should delete it
    EthernetPacket retval;

    data_outgoing_ethernet.construct_data();
    uint8_t* packet_buffer = data_outgoing_ethernet.data();

    memcpy(retval.payload.data, packet_buffer, data_outgoing_ethernet.size());

    retval.header.payload_size = data_outgoing_ethernet.size();
    return retval;
}

void CommsLayer::decode(EthernetPacket source_packet) {
    // reset outgoing data
    // memset(&data_incoming_ethernet, 0, sizeof(FirmwareData));
    data_incoming_ethernet = HiveData();
    // decode and push into queue data_incoming_ethernet
    bool end_of_transmission = false;
    int cur_offset = 0;
    int num_items = 0;
    while (!end_of_transmission) {
        // if we are too far into packet, abort!
        if (ETHERNET_PACKET_PAYLOAD_MAX_SIZE - cur_offset <= sizeof(CommsData)) {
            end_of_transmission = true;
            break;
        }

        // grab one commsdata, validate, then add its general info
        CommsData* cur_metadata = (CommsData*)(&source_packet.payload.data[cur_offset]);

        if (cur_metadata->size == 0) {
            end_of_transmission = true;
            break;
        }

        switch (cur_metadata->type_label) {
        case TypeLabel::TargetState:
            data_incoming_ethernet.target_state = *((RobotState*)(cur_metadata));
            break;
        case TypeLabel::FWSample2:
            data_incoming_ethernet.override_state = *((RobotState*)(cur_metadata));
            break;
        default:
            printf("CommsLayer::NOTICE: received data in Ethernet packet of unknown or unsupported type for HiveData megastruct\n");
            break;
        }

        cur_offset += cur_metadata->size;
        num_items++;
    }
    // printf("number of items in transmission: %d\n", num_items);
}




} // namespace Comms    