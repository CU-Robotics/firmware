#ifndef CONFIG_LAYER
#define CONFIG_LAYER

#include "usb_hid.hpp"

#define CONFIG_LAYER_DEBUG

/// @brief arbitrary cap on config packets that can be received (make sure it's enough)
const int MAX_CONFIG_PACKETS = 64;

class ConfigLayer {
private:
    /// @brief array to save config packets
    CommsPacket config_packets[MAX_CONFIG_PACKETS];

    /// @brief flag indicating if all config packets have been received
    bool configured = false;

    /// @brief current YAML section we are seeking
    int seek_sec = -1;

    /// @brief current YAML subsection we are seeking
    int seek_subsec = 0;

    /// @brief number of YAML sections
    uint16_t num_sec;

    /// @brief array to store number of subsections per YAML section
    uint8_t subsec_sizes[MAX_CONFIG_PACKETS] = { 0 };

public:
    /// @brief default constructor
    ConfigLayer() {}

    /// @brief check incoming packet from the comms layer and update outgoing packet accordingly to request next config packet
    /// @param in incoming comms packet
    /// @param out outgoing comms packet to write config requests to
    void process(CommsPacket *in, CommsPacket *out);

    /// @brief return configured flag (check if all config packets have been received)
    /// @return the configured flag
    bool is_configured() { return configured; }
};

#endif