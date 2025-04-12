#pragma once 

#include "comms/data/comms_data.hpp"        // for CommsData
#include "comms/hid_comms.hpp"              // for HIDComms
#include "comms/ethernet_comms.hpp"         // for EthernetComms
#include "comms/data/packet_payload.hpp"    // for PacketPayload

namespace Comms {

/// @brief The CommsLayer class is the top-level class for the communication layer. Handles HID and Ethernet communications
class CommsLayer {
public:
    /// @brief Defualt constructor
    CommsLayer();

    /// @brief Destructor
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

    /// @brief Prepare and send packets to the appropriate physical layer
    /// @note This function should only be ran internally
    void send_packets();

    /// @brief Receive and process packets from the appropriate physical layer
    /// @note This function should only be ran internally
    void recv_packets();

    /// @brief Check if Ethernet is connected
    /// @return True if connected, false if not
    bool is_ethernet_connected();

    /// @brief Check if HID is connected
    /// @return True if connected, false if not
    bool is_hid_connected();

    /// @brief Clear all physical layer outgoing buffers
    void clear_outgoing_buffers();

public:
    /// @brief Get the outgoing ethernet packet
    /// @return The outgoing ethernet packet
    EthernetPacket get_ethernet_outgoing();
    /// @brief Get the outgoing HID packet
    /// @return The outgoing HID packet
    HIDPacket get_hid_outgoing();
    /// @brief Set the incoming ethernet packet
    /// @param packet The incoming ethernet packet
    void set_ethernet_incoming(EthernetPacket&& packet);
    /// @brief Set the incoming HID packet
    /// @param packet The incoming HID packet
    void set_hid_incoming(HIDPacket&& packet);

    /// @brief Get the firmware data
    /// @return The firmware data
    FirmwareData& get_firmware_data();
    /// @brief Set the firmware data
    /// @param data The firmware data to set
    void set_firmware_data(FirmwareData& data);

    /// @brief Get the hive data
    /// @return The hive data
    HiveData& get_hive_data();
    /// @brief Set the hive data
    /// @param data The hive data to set
    void set_hive_data(HiveData& data);

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
    /// @brief Incoming ethernet packet
    EthernetPacket m_ethernet_incoming;
    /// @brief Outgoing ethernet packet
    EthernetPacket m_ethernet_outgoing;

    /// @brief HID physical layer
    Comms::HIDComms m_hid;
    /// @brief Incoming HID packet
    HIDPacket m_hid_incoming;
    /// @brief Outgoing HID packet
    HIDPacket m_hid_outgoing;

    /// @brief Packet payload for Ethernet
    PacketPayload m_ethernet_payload{ETHERNET_PACKET_PAYLOAD_SIZE};
    /// @brief Packet payload for HID
    PacketPayload m_hid_payload{HID_PACKET_PAYLOAD_SIZE};

    /// @brief Hive data
    HiveData m_hive_data;

    /// @brief Firmware data
    FirmwareData m_firmware_data;

};

}   // namespace Comms

extern Comms::CommsLayer comms_layer;