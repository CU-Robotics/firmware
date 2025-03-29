#pragma once 

#include "comms/data/comms_data.hpp"        // CommsData
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

    /// @brief Process the HID layer by sending, receiving, and processing HID packets
    void process_hid_layer();

    /// @brief Process the Ethernet layer by sending, receiving, and processing Ethernet packets
    void process_ethernet_layer();

    /// @brief Check if Ethernet is connected
    /// @return True if connected, false if not
    bool is_ethernet_connected();

    /// @brief Check if HID is connected
    /// @return True if connected, false if not
    bool is_hid_connected();

    /// @brief Get the hive data
    /// @return The hive data
    HiveData& get_hive_data();

    /// @brief Set the hive data
    /// @param data The hive data to set
    void set_hive_data(HiveData& data);

    /// @brief Get the firmware data
    /// @return The firmware data
    FirmwareData& get_firmware_data();

    /// @brief Set the firmware data
    /// @param data The firmware data to set
    void set_firmware_data(FirmwareData& data);

    // TODO: remove these sometime soon
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
    Comms::EthernetComms m_ethernet;
    /// @brief Ethernet outgoing packet
    EthernetPacket m_ethernet_outgoing;

    /// @brief HID physical layer
    Comms::HIDComms m_hid;
    /// @brief HID outgoing packet
    HIDPacket m_hid_outgoing;

    /// @brief Packet payload for Ethernet
    PacketPayload m_ethernet_payload{ETHERNET_PACKET_PAYLOAD_MAX_SIZE};
    /// @brief Packet payload for HID
    PacketPayload m_hid_payload{HID_PACKET_PAYLOAD_SIZE};

    /// @brief Hive data
    HiveData m_hive_data;

    /// @brief Firmware data
    FirmwareData m_firmware_data;

};

}   // namespace Comms

extern Comms::CommsLayer comms_layer;