#ifndef USB_HID_HPP
#define USB_HID_HPP

#include "Arduino.h"
#include "usb_rawhid.h"				// usb_rawhid functions
#include "../controls/state.hpp"	// STATE_LEN macro

/// @brief Packet size for communication packets
constexpr unsigned int COMMS_PACKET_SIZE = 1023u;

// TODO: make this dynamic and grabbed from the config packet
// Khadas -> Teensy
/// @brief The offset of the Packet ID from the base of the Khadas packet
constexpr unsigned int KHADAS_PACKET_ID_OFFSET = 1u; 	// 2 bytes
/// @brief The offset of the Packet info bits from the base of the Khadas packet
constexpr unsigned int KHADAS_PACKET_INFO_OFFSET = 3u;	// 1 byte
/// @brief The offset of the Packet target state from the base of the Khadas packet
constexpr unsigned int KHADAS_PACKET_TSTATE_OFFSET = 4u;	// 288 bytes
/// @brief The offset of the Packet ref draw data from the base of the Khadas packet
constexpr unsigned int KHADAS_PACKET_REF_OFFSET = 292u;	// 128 bytes
/// @brief The offset for the request byte if hive sent an override state
constexpr unsigned int KHADAS_PACKET_HIVE_OVERRIDE_STATE_REQUEST_OFFSET = 420u; // 1 byte
/// @brief The offset of the override state from hive
constexpr unsigned int KHADAS_PACKET_HIVE_OVERRIDE_STATE_OFFSET = 421u; // 288 bytes
/// @brief The offset to the end of the Khadas packet
constexpr unsigned int KHADAS_PACKET_END_OFFSET = 709u;


// Teensy -> Khadas
/// @brief The offset of the packet ID from the base of the Teesny packet
constexpr unsigned int TEENSY_PACKET_ID_OFFSET = 1u;	// 2 bytes
/// @brief The offset of the packet info bits from the base of the Teesny packet
constexpr unsigned int TEENSY_PACKET_INFO_OFFSET = 3u;	// 1 bytes
/// @brief The offset of the packet time from the base of the Teesny packet
constexpr unsigned int TEENSY_PACKET_TIME_OFFSET = 4u;	// 8 bytes
/// @brief The offset of the packet estimated state from the base of the Teesny packet
constexpr unsigned int TEENSY_PACKET_ESTATE_OFFSET = 12u;	// 288 bytes
/// @brief The offset of the packet sensor data from the base of the Teesny packet
constexpr unsigned int TEENSY_PACKET_SENSOR_OFFSET = 300u;	// 400 bytes
/// @brief The offset of the packet ref data from the base of the Teesny packet
constexpr unsigned int TEENSY_PACKET_REF_OFFSET = 700u;	// 180 bytes
/// @brief The offset to the end of the Teensy packet
constexpr unsigned int TEENSY_PACKET_END_OFFSET = 880u;

/// @brief The offset to dr16 data from the sensor data section
constexpr unsigned int SENSOR_DR16_OFFSET = 0u;
/// @brief The offset to the lidar1 data from the sensor data section
constexpr unsigned int SENSOR_LIDAR1_OFFSET = 18u;
/// @brief The offset to the lidar2 data from the sensor data section
constexpr unsigned int SENSOR_LIDAR2_OFFSET = 172u;

/// @brief An encapsulating data struct managing data from all of Teensy's sensors
struct SensorData {
	/// @brief Default constructor
	SensorData() = default;
	/// @brief Constructs a SensorData struct with a raw byte array of data
	/// @param raw_data Raw byte array of data
	SensorData(char* raw_data) {
		memcpy(raw, raw_data, sizeof(SensorData));
	}

	/// @brief Raw byte array
	char raw[400] = { 0 };
};

/// @brief An encapsulating data struct managing a HID packet
struct CommsPacket {
	/// @brief The raw array of bytes of a packet
	char raw[COMMS_PACKET_SIZE] = { 0 };

	// common getters
	/// @brief Get the ID of this packet
	/// @return Packet ID
	uint8_t get_id();
	/// @brief Get the info bits of this packet
	/// @return The packet info bits
	uint8_t get_info();

	// common setters
	/// @brief Set the ID of this packet
	/// @param id The packet ID
	void set_id(uint16_t id);
	/// @brief Set the info bits of this packet
	/// @param info The packet info bits
	void set_info(uint8_t info);

	// khadas getters
	/// @brief Get the target state from this packet
	/// @param state The float array to put the state into
	void get_target_state(float state[STATE_LEN][3]);
	/// @brief Get the ref draw data from this packet
	/// @param draw_data The char array to put the data into
	void get_ref_draw_data(char** draw_data);

	/// @brief Get the hive override request bit
	/// @return The request bit
	uint8_t get_hive_override_request();

	/// @brief Get the hive override state
	/// @param state The float array to put the state into
	void get_hive_override_state(float state[STATE_LEN][3]);

	// teensy setters
	/// @brief Set the time of this packet
	/// @param time The time as a double
	void set_time(double time);
	/// @brief Set the estimated state for this packet
	/// @param state The float array of state
	void set_estimated_state(float state[STATE_LEN][3]);
	/// @brief Set the sensor data for this packet
	/// @param sensor_data The sensor data struct reference to use
	void set_sensor_data(SensorData* sensor_data);
	/// @brief Set the ref data for this packet
	/// @param ref_data The ref data byte array
	void set_ref_data(uint8_t ref_data[180]);
};

/// @brief The communications layer between Khadas and Teensy
class HIDLayer {
public:
	/// @brief Default constructor
	HIDLayer();

	/// @brief Initialize the HID 
	void init();

	/// @brief Attempt to read and write a packet to Khadas
	void ping();

	/// @brief Print the outgoing packet
	/// @note This massively slows the loop down
	void print_outgoing();

	/// @brief Print the incomming packet
	/// @note This massively slows the loop down
	void print_incoming();

	/// @brief Get the packet comming from Khadas
	/// @return A pointer to the received Khadas packet
	inline CommsPacket* get_incoming_packet() { return &m_incomingPacket; }
	/// @brief Get the packet to Khadas
	/// @return A pointer to the packet to be sent to Khadas
	inline CommsPacket* get_outgoing_packet() { return &m_outgoingPacket; }

private:
	/// @brief Attempt a read on HID
	/// @return True/False on read success
	bool read();
	/// @brief Attempt a write on HID
	/// @return True/False on write success
	bool write();

private:
	/// @brief An encapsulating struct around the packet received from Khadas
	CommsPacket m_incomingPacket{};
	/// @brief An encapsulating struct around the packet to be sent to Khadas
	CommsPacket m_outgoingPacket{};
	/// @brief An encapsulating struct around all the sensor data to be sent to Khadas
	SensorData m_sensorData{};

	/// @brief Counter on how many packets have been received
	long long unsigned m_packetsRead = 0;
	/// @brief Counter on how many packets have been sent
	long long unsigned m_packetsSent = 0;
	/// @brief Counter on how many packets have failed to be sent
	/// @note This is only incremented on a failed write, not read
	long long unsigned m_packetsFailed = 0;
};

extern HIDLayer comms;

#endif // end USB_HID_HPP