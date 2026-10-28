#pragma once
#include <vector>
#include "comms/data/comms_data.hpp"
namespace Comms {
struct CommsLayer {
    std::vector<std::vector<uint8_t>> queued;
    void queue_data(const CommsData* data) {
        const auto* bytes = reinterpret_cast<const uint8_t*>(data);
        queued.emplace_back(bytes, bytes + data->size);
    }
};
inline CommsLayer comms_layer;
}
