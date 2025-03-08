#pragma once 

#include "comms/data/comms_data.hpp"
#include "comms/hid_comms.hpp"              // HIDComms
#include "comms/ethernet_comms.hpp"         // EthernetComms
#include "comms/data/packet_payload.hpp"    // PacketPayload

namespace Comms {

/// @brief The CommsLayer class is the top-level class for the communication layer. Handles HID and Ethernet communications
class CommsLayer {
public:
    // Default constructor
    CommsLayer();

    // Destructor
    ~CommsLayer();

public:
    /// @brief Initializes the CommsLayer
    /// @return Exit status, 0 for success, < 0 for error
    int init();

    /// @brief Starts the CommsLayer, runs the Ethernet and HID communication modules
    /// @return Exit status, 0 for success, < 0 for error
    /// @note Should never return
    int run();

public:
    /// @brief Send a CommsData packet to the appropriate packet payload
    /// @param data The CommsData packet to send
    void queue_data(CommsData* data);

    HIDPacket get_hid_incoming();
    void set_hid_outgoing(HIDPacket& packet);
    EthernetPacket get_ethernet_incoming();
    void set_ethernet_outgoing(EthernetPacket& packet);
    
private:
    /// @brief Initializes HID and starts its thread
    /// @return True if successful, false if failed
    bool initialize_hid();

    /// @brief Initializes Ethernet and starts its thread
    /// @return True if successful, false if failed
    bool initialize_ethernet();

private:
    /// @brief Ethernet physical layer
    EthernetComms m_ethernet;
    EthernetPacket m_ethernet_outgoing;

    /// @brief HID physical layer
    // TODO: hid comms namespace
    HIDComms m_hid;
    HIDPacket m_hid_outgoing;

    /// @brief Packet payload for Ethernet
    PacketPayload m_ethernet_payload{ETHERNET_PACKET_PAYLOAD_MAX_SIZE};
    /// @brief Packet payload for HID
    PacketPayload m_hid_payload{HID_PACKET_SIZE};

};

}   // namespace Comms

extern Comms::CommsLayer comms_layer;