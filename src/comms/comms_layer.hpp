#pragma once

#include "ethernet_comms.hpp"
#include "data/firmware_data.hpp"
#include "data/hive_data.hpp"

#define MAX_ETHERNET_PACKETS 128        // TODO: stop making up numbers


// purpose: translation between data structs and byte streams for transmission over physical layers (Ethernet)

namespace Comms {

// array of ethernet packets for combined transmission
struct EthernetPackage {
    EthernetPackage();
    EthernetPacket packets[MAX_ETHERNET_PACKETS];
    uint8_t num_packets;

    // these types are shared among all packets
    EthernetPacketFlags flag;
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
    EthernetPackage encode(FirmwareData data, int data_type = DATA, int data_flag = NORMAL);

    // take ethernet packet sequence, convert it into FirmwareData
    HiveData decode(EthernetPackage payload);

    // transmit a given EthernetPackage
    int transmit(EthernetPackage payload);

    // receive an EthernetPackage
    // nullptr if failed, else success
    EthernetPackage receive();

public:
// - purpose-built comms functions for complex use cases

    // config functions:

    // send full configuration packet over Ethernet, returns < 0 on failure, 0 on success
    int teensy_configure();

private:

    // constructs a packet based on input data
    EthernetPacket construct_packet(uint8_t* bytes, uint16_t payload_size, uint8_t type, uint8_t flag);

    // construct a packet to determine EOT
    EthernetPacket construct_EOT_packet();

    // number of packets sent in total
    uint16_t sequence;
};

} // namespace Comms
