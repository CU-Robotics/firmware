#pragma once

#include "ethernet_comms.hpp"
#include "data/comms_data.hpp"
#include "data/packet_payload.hpp"

#define MAX_ETHERNET_PACKETS    128        // TODO: stop making up numbers
#define MAX_HID_PACKETS         128


namespace Comms {

class CommsLayer {
public:
// - init functions

    CommsLayer() = default;         // default constructor
    ~CommsLayer() = default;        // destructor

    int init();
    int init(bool e_Ethernet, bool e_HID);
    int loop();

private:
// - physical layers

    EthernetComms Ethernet;     // Ethernet physical layer

public:
// - data I/O functions

    // give CommsLayer a piece of data to send out
    void send(CommsData* data, PhysicalMedium medium);

    // pull a piece of data that CommsLayer is ready to provide
    HiveData receive(PhysicalMedium medium);

    EthernetPacket encode();

    void decode(EthernetPacket source_packet);


private:

    // data incoming to be output, TODO make this a queue so we can store many data packets! (or probably a LL)
    HiveData data_incoming_ethernet;

    PacketPayload data_outgoing_ethernet;


    // sequence number
    uint32_t sequence;

    bool enable_Ethernet;
    bool enable_HID;
};

} // namespace Comms
