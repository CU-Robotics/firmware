#pragma once

#include "ethernet_comms.hpp"
#include "data/comms_data.hpp"

#define MAX_ETHERNET_PACKETS 128        // TODO: stop making up numbers


namespace Comms {

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

    // give CommsLayer a piece of data to send out
    void send(CommsData* data, PhysicalMedium medium);

    // pull a piece of data that CommsLayer is ready to provide
    HiveData receive(PhysicalMedium medium);
    

private:

    // data incoming to be output, TODO make this a queue so we can store many data packets! (or probably a LL)
    CommsData* data_incoming_ethernet;
    CommsData* data_incoming_HID;

    // data outgoing to be sent over a particular medium, TODO make this a multi-layer queue
    CommsData* data_outgoing_ethernet;
    CommsData* data_outgoing_HID;


    // sequence number
    uint32_t sequence;

    
};

} // namespace Comms
