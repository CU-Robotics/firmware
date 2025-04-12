#ifndef HID_COMMS_HPP
#define HID_COMMS_HPP

#include "Arduino.h"
#include "usb_rawhid.h"				// for usb_rawhid
#include "hid_packet.hpp"			// for HIDPacket

namespace Comms {

/// @brief The communications layer between Hive and Teensy
class HIDComms {
public:
	/// @brief Default constructor
	HIDComms();

public:
	/// @brief Initialize the HID 
	void init();

    /// @brief Send an HID packet to Hive
    /// @param packet The packet to send
    /// @return True if successful, false if failed
    bool send_packet(HIDPacket& packet);

    /// @brief Receive an HID packet from Hive
    /// @param packet The packet to fill with data
    /// @return True if successful, false if failed
    bool recv_packet(HIDPacket& packet);

	/// @brief Get the connection status
    /// @return True if connected, false if not
    bool is_connected() const;

    /// @brief Get the current initialized status of the HID layer
    /// @return True if the HID layer is initialized
    bool is_initialized();

	/// @brief Print the outgoing packet
	/// @note This massively slows the loop down
	void print_outgoing();

	/// @brief Print the incomming packet
	/// @note This massively slows the loop down
	void print_incoming();

private:
	/// @brief An encapsulating struct around the packet received from Khadas
	HIDPacket m_incomingPacket{};
	/// @brief An encapsulating struct around the packet to be sent to Khadas
	HIDPacket m_outgoingPacket{};

	/// @brief Counter on how many packets have been received
	long long unsigned m_packetsRead = 0;
	/// @brief Counter on how many packets have been sent
	long long unsigned m_packetsSent = 0;
	/// @brief Counter on how many packets have failed to be sent
	/// @note This is only incremented on a failed write, not read
	long long unsigned m_packetsFailed = 0;
};

extern HIDComms comms;

} 	// namespace Comms

#endif	// HID_COMMS_HPP