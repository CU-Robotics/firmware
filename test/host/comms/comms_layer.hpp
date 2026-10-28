#pragma once

namespace Comms {
struct CommsData;

/// @brief Host tests do not transmit Ethernet/HID telemetry.
struct CommsLayer {
    void queue_data(const CommsData*) {}
};

inline CommsLayer comms_layer;
} // namespace Comms
