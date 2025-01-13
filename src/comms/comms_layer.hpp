#pragma once

#include "ethernet_comms.hpp"
#include "data/firmware_data.hpp"
#include "data/hive_data.hpp"

#define MAX_ETHERNET_PACKETS 128        // TODO: stop making up numbers


// purpose: translation between data structs and byte streams for transmission over physical layers (Ethernet)

namespace Comms {

// EthernetPayload: contains information about a payload consisting of several Ethernet packets
// we need this because we have to account for the fact that transmissions over Ethernet could (generally)
// require several packets, so this struct encompasses all packets needed to transmit a data type
// more physical comms methods (such as HID) can be included by adding support for more payload types

struct EthernetPayload {
    EthernetPacket* packets;        // NOTE: if we use malloc() for this, be really careful and dont be stupid
    unsigned int length;
    // unsigned int section_sizes[MAX_ETHERNET_PACKETS];   // sizes of each section of data for our packets
    int type;                       // set EthernetPacket type (included in packets)
    int flag;                       // set EthernetPacket flag (included in packets)

    EthernetPayload();
    ~EthernetPayload();
};



class CommsLayer {
public:
// - init functions

    CommsLayer() = default;         // default constructor
    ~CommsLayer() = default;        // destructor

    int init();
    int loop();

private:
// - physical layers

    EthernetComms Ethernet;     // Ethernet physical layer

public:
// - data I/O functions

    // take data, convert it into ethernet compatible form (packet sequence)
    // EthernetPayload will incorporate the (optional) given data_type and data_flag, passing them through
    // to the packet headers individually
    EthernetPayload encode(FirmwareData data, int data_type = DATA, int data_flag = NORMAL);

    // take ethernet packet sequence, convert it into FirmwareData
    HiveData decode(EthernetPayload payload);

    // transmit a given EthernetPayload
    int transmit(EthernetPayload* payload);

    // receive an EthernetPayload
    // nullptr if failed, else success
    EthernetPayload* receive();

public:
// - purpose-built comms functions for complex use cases

    // config functions:

    // send full configuration packet over Ethernet, returns < 0 on failure, 0 on success
    int teensy_configure();

private:

    // constructs a packet based on input data
    EthernetPacket construct_packet(uint8_t* bytes, uint16_t payload_size, uint8_t type, uint8_t flag, uint64_t timestamp);

    // construct a packet to determine EOT
    EthernetPacket construct_EOT_packet();

    // receives ethernet packet and removes from m_incoming ethernet packet buffer
    EthernetPacket ethernet_packet_receive();

    bool ethernet_is_valid_packet(EthernetPacket packet);

    // number of packets sent in total
    uint16_t sequence;
};

} // namespace Comms
