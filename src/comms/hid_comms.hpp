#ifndef HID_COMMS_HPP
#define HID_COMMS_HPP

#include "Arduino.h"
#include "usb_rawhid.h"				// usb_rawhid functions
#include "../controls/state.hpp"	// STATE_LEN macro
#include <optional>

#include "hid_packet.hpp"

namespace Comms {

/// @brief The communications layer between Khadas and Teensy
class HIDComms {
public:
	/// @brief Default constructor
	HIDComms();

	/// @brief Initialize the HID 
	void init();

	/// @brief Attempt to read and write a packet to Khadas
	std::optional<HIDPacket> sendReceive(HIDPacket& outgoing_packet);

	/// @brief Print the outgoing packet
	/// @note This massively slows the loop down
	void print_outgoing();

	/// @brief Print the incomming packet
	/// @note This massively slows the loop down
	void print_incoming();

	/// @brief Get the packet comming from Khadas
	/// @return A pointer to the received Khadas packet
	inline HIDPacket get_incoming_packet() { return m_incomingPacket; }

	/// @brief Set the packet to be sent to Khadas
	/// @param packet The packet to send
	inline void set_outgoing_packet(HIDPacket& packet) { m_outgoingPacket = packet; }

private:
	/// @brief Attempt a read on HID
	/// @return True/False on read success
	bool read(HIDPacket& incoming_packet);
	/// @brief Attempt a write on HID
	/// @return True/False on write success
	bool write(HIDPacket& outgoing_packet);

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