#include "packet_payload.hpp"

#include <algorithm> // for min
#include <cassert>

namespace Comms {

PacketPayload::PacketPayload(uint16_t max_data_size) {
    this->max_data_size = max_data_size;
    raw_data = new uint8_t[max_data_size];

    clear();
}

PacketPayload::~PacketPayload() {
    // frees memory used by raw_data in constructor
    delete[] raw_data;
}

void PacketPayload::construct_data() {
    clear();

    append_data_from_queue(high_priority_send_queue);
    append_data_from_queue(medium_priority_send_queue);

    // fill_logging_data_from_queue(logging_send_queue);
}

bool PacketPayload::add(CommsData* data) {
    switch (data->priority) {
    case Priority::High: {
        if(high_priority_send_queue.size() >= MAX_QUEUE_SIZE) return false;
        high_priority_send_queue.push(data);
        return true;
        break;
    } case Priority::Medium: {
        if(medium_priority_send_queue.size() >= MAX_QUEUE_SIZE) return false;
        medium_priority_send_queue.push(data);
        return true;
        break;
    }
    // } case Priority::Logging: {
    //     // a logging priority item must be of type logging data since we grab data from logs dynamically (handled in this file)
    //     LoggingData* logging = static_cast<LoggingData*>(&data);
    //     logging_send_queue.push(logging);
    //     break;
    }
    return false;
}

uint8_t* PacketPayload::data() {
    return raw_data;
}

uint16_t PacketPayload::size() {
    return max_data_size - remaining_size;
}

void PacketPayload::clear() {
    // fill raw data with 0's, and reset remaining size
    memset(raw_data, 0, max_data_size);
    remaining_size = max_data_size;
}

void PacketPayload::append_data_from_queue(std::queue<CommsData*>& queue) {
    while (!queue.empty() && remaining_size > 0) {
        CommsData* next_data = (queue.front());

        bool successful_append = try_append_data(next_data);

        if (successful_append) {
            delete queue.front();   // this is expected to be heap-allocated
            queue.pop(); // remove the appended item from the queue, going to the next.
        } else {
            // If we could not append (out of space), give up.
            // Could optimize the space by continuing to try every element in queue instead.
            break;
        }
    }
}

bool PacketPayload::try_append_data(CommsData* data) {
    // if we don't have enough space left in our packet to store this data
    if (data->size > remaining_size) {
        return false; // failure

    }
    // we have enough space, append it

    // where to start the append
    uint16_t append_offset = max_data_size - remaining_size;

    // append into raw data buffer.
    memcpy(&raw_data[append_offset], data, data->size);

    remaining_size -= data->size;

    return true; // success
}

// void PacketPayload::fill_logging_data_from_queue(std::queue <LoggingData*>& queue) {
//     while (!queue.empty() && remaining_size > 0) {
//         LoggingData next_data = *(queue.front());

//         bool finished_append = try_append_splittable_logging_data(next_data);

//         if (finished_append) {
//             // if we have appended the entirety of the splittlable comms data
//             queue.pop(); // remove the appended item from the queue, going to the next.
//         } else {
//             // If we could append the full amount (out of space), give up.
//             break;
//         }
//     }
// }

// bool PacketPayload::try_append_splittable_logging_data(LoggingData& log) {
//     // if an empty (except for Header) CommsData doesn't even fit
//     if (sizeof(CommsData) > remaining_size) {
//         return false; // fail
//     }

//     // where to start the append
//     uint16_t append_offset = max_data_size - remaining_size;

//     // we want to fill as much as possible:
//     // either the rest of the requested data, or the entirety of the remaining size
//     uint16_t fill_size = std::min(remaining_size, log.remaining_size_to_serialize());

//     // append into raw buffer using log's serialize (this is why had to downcast)
//     log.serialize(raw_data + append_offset, fill_size);
//     remaining_size -= fill_size;

//     if (log.all_data_has_been_serialized()) {
//         // we filled up the rest of the requested logs, we can say we finished
//         return true;
//     }

//     // we may have successfully appended some data,
//     // but there is still more data in this LoggingData, so we don't want to say to pop it yet.
//     return false;
// }




}