#include "packet_payload.hpp"
#include "safety.hpp"

#include <algorithm>                        // for min

#if defined(HIVE)
#include <iostream>                         // for std::cout
#include "modules/comms/comms_layer.hpp"    // for CommsLayer
#include <mutex>                            // for std::lock_guard, std::mutex
#elif defined(FIRMWARE)
#include "comms/comms_layer.hpp"            // for CommsLayer
#endif

namespace Comms {

PacketPayload::PacketPayload(uint16_t max_data_size) {
    this->max_data_size = max_data_size;
    raw_data = new uint8_t[this->max_data_size];

    clear_raw_data();
}

PacketPayload::~PacketPayload() {
    delete[] raw_data;

    while (!high_priority_send_queue.empty()) {
        delete high_priority_send_queue.front();
        high_priority_send_queue.pop();
    }

    while (!medium_priority_send_queue.empty()) {
        delete medium_priority_send_queue.front();
        medium_priority_send_queue.pop();
    }
}

void PacketPayload::construct_data() {
    clear_raw_data();

    append_data_from_queue(high_priority_send_queue);
    append_data_from_queue(medium_priority_send_queue);

}

void PacketPayload::deconstruct_data(uint8_t* data, uint16_t size) {
    safety::assert_or_safety_procedure(size == max_data_size, "PacketPayload::deconstruct_data: Data size %u does not match max data size %u", size, max_data_size);

    

    uint16_t offset = 0;

    while (1) {
        // get the header
        CommsData* header = reinterpret_cast<CommsData*>(data + offset);

        // increment the data pointer
        offset += header->size;

        Serial.printf("Recieved size: %u, type: %s, offset: %u\n", header->size, to_string(header->type_label).c_str(), offset);

        // if the header is a NONE type, we are done
        if (header->type_label == TypeLabel::NONE) {
            break;
        }
        
        // send the data to the mega struct
        place_incoming_data_in_mega_struct(header);

        // if we have reached the end of the data, we are done
        if (offset >= size - sizeof(CommsData)) {
            break;
        }
    }    
}

void PacketPayload::add(CommsData* data) {
    switch (data->priority) {
        case Priority::High: {
            if (high_priority_send_queue.size() < MAX_QUEUE_SIZE) {
                high_priority_send_queue.push(data);
                break;
            } else {
                // since we could not successfully add to the high priority queue,
                // we intentionally fall through to the medium priority queue
                [[fallthrough]];
            }
        } case Priority::Medium: {
            if (medium_priority_send_queue.size() < MAX_QUEUE_SIZE) {
                medium_priority_send_queue.push(data);
            } else {
                delete data;
            }
            break;    
        }
        // Don't have a default case so that the compiler will warn us if we forget to handle a priority case
    }
}
uint8_t* PacketPayload::data() {
    return raw_data;
}

void PacketPayload::clear_queues() {
    // clear the queues
    while (!high_priority_send_queue.empty()) {
        delete high_priority_send_queue.front();
        high_priority_send_queue.pop();
    }

    while (!medium_priority_send_queue.empty()) {
        delete medium_priority_send_queue.front();
        medium_priority_send_queue.pop();
    }
    // clear the raw data buffer
    clear_raw_data();
}

uint16_t PacketPayload::get_high_priority_queue_size() const {
    return high_priority_send_queue.size();
}

uint16_t PacketPayload::get_medium_priority_queue_size() const {
    return medium_priority_send_queue.size();
}

uint16_t PacketPayload::get_max_size() const {
    return max_data_size;
}

void PacketPayload::clear_raw_data() {
    // fill raw data with 0's, and reset remaining size
    memset(raw_data, 0, max_data_size);
    remaining_data_size = max_data_size;
}

void PacketPayload::append_data_from_queue(std::queue<CommsData*>& queue) {
    while (!queue.empty() && remaining_data_size > 0) {
        CommsData* next_data = queue.front();

        bool successful_append = try_append_data(next_data);

        if (successful_append) {
            // set the data in the mega struct
            place_outgoing_data_in_mega_struct(next_data);
            
            // free the pointer
            delete next_data;
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
    if (data->size > remaining_data_size) {
        return false; // failure
    }
    // we have enough space, append it

    // where to start the append
    uint16_t append_offset = max_data_size - remaining_data_size;

    // append into raw data buffer.
    memcpy(raw_data + append_offset, data, data->size);

    remaining_data_size -= data->size;

    return true; // success
}

void PacketPayload::place_incoming_data_in_mega_struct(CommsData* data) {
    HiveData& hive_data = comms_layer.get_hive_data();
    
    // Serial.printf("Placing incoming in mega struct: %s\n", to_string(data->type_label).c_str());
    hive_data.set_data(data);
}

void PacketPayload::place_outgoing_data_in_mega_struct(CommsData* data) {

    FirmwareData& firmware_data = comms_layer.get_firmware_data();

    // Serial.printf("Placing outgoing in mega struct: %s\n", to_string(data->type_label).c_str());
    firmware_data.set_data(data);
}

}   // namespace Comms
