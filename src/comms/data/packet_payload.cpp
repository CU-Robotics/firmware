#include "packet_payload.hpp"
#include "utils/safety.hpp"
#include "comms/comms_layer.hpp"

#include <cstring>
#include <limits>

namespace Comms {

PacketPayload::PacketPayload(uint8_t* high_priority_buffer, uint8_t* medium_priority_buffer, uint16_t max_data_size)
    : max_data_size(max_data_size), high_priority_buf(high_priority_buffer), medium_priority_buf(medium_priority_buffer) {
}

uint16_t PacketPayload::pack_staged_buffer(uint8_t* source, uint16_t& used, uint16_t& count, uint8_t* destination, uint16_t destination_capacity) {
    uint16_t emitted_bytes = 0;
    uint16_t emitted_count = 0;

    while (emitted_bytes < used) {
        const uint16_t staged_bytes_remaining = used - emitted_bytes;
        if (staged_bytes_remaining < sizeof(CommsData)) {
            used = 0;
            count = 0;
            safety::assert_or_safety_procedure(false, "PacketPayload::construct_data: Staged record header is truncated");
            return 0;
        }

        CommsData header;
        memcpy(&header, source + emitted_bytes, sizeof(header));
        if (header.size < sizeof(CommsData) || header.size > staged_bytes_remaining || header.size > max_data_size) {
            const uint16_t invalid_size = header.size;
            used = 0;
            count = 0;
            safety::assert_or_safety_procedure(false, "PacketPayload::construct_data: Invalid staged record size %u with %u bytes remaining and %u byte capacity", invalid_size, staged_bytes_remaining, max_data_size);
            return 0;
        }

        if (header.size > destination_capacity - emitted_bytes) {
            break;
        }

        emitted_bytes += header.size;
        emitted_count++;
    }

    if (emitted_count > count) {
        used = 0;
        count = 0;
        safety::assert_or_safety_procedure(false, "PacketPayload::construct_data: Staged record count is corrupt");
        return 0;
    }

    if (emitted_bytes > 0) {
        memcpy(destination, source, emitted_bytes);
    }

    const uint16_t retained_bytes = used - emitted_bytes;
    if (retained_bytes > 0 && emitted_bytes > 0) {
        memmove(source, source + emitted_bytes, retained_bytes);
    }
    used = retained_bytes;
    count -= emitted_count;

    return emitted_bytes;
}

uint16_t PacketPayload::construct_data(uint8_t* destination) {
    if (destination == nullptr) {
        safety::assert_or_safety_procedure(false, "PacketPayload::construct_data: Destination is null");
        return 0;
    }

    uint16_t written = pack_staged_buffer(high_priority_buf, high_priority_used, high_priority_count, destination, max_data_size);
    written += pack_staged_buffer(medium_priority_buf, medium_priority_used, medium_priority_count, destination + written, max_data_size - written);

    const uint16_t remaining = max_data_size - written;
    if (remaining >= sizeof(CommsData)) {
        const CommsData sentinel{};
        memcpy(destination + written, &sentinel, sizeof(sentinel));
    }

    return written;
}

void PacketPayload::deconstruct_data(uint8_t* data, uint16_t size) {
    safety::assert_or_safety_procedure(size == max_data_size, "PacketPayload::deconstruct_data: Data size %u does not match max data size %u", size, max_data_size);

    uint16_t offset = 0;

    while (1) {
        // get the header
        CommsData* header = reinterpret_cast<CommsData*>(data + offset);

        // increment the data pointer
        offset += header->size;
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

bool PacketPayload::add(const CommsData* data) {
    if (data == nullptr) {
        safety::assert_or_safety_procedure(false, "PacketPayload::add: Data is null");
        return false;
    }
    if (data->size < sizeof(CommsData) || data->size > max_data_size) {
        safety::assert_or_safety_procedure(false, "PacketPayload::add: Invalid record size %u for %u byte capacity", data->size, max_data_size);
        return false;
    }

    uint8_t* destination = nullptr;
    uint16_t* used = nullptr;
    switch (data->priority) {
    case Priority::High:
        if (data->size <= max_data_size - high_priority_used) {
            destination = high_priority_buf;
            used = &high_priority_used;
            break;
        }
        [[fallthrough]];
    case Priority::Medium:
        if (data->size <= max_data_size - medium_priority_used) {
            destination = medium_priority_buf;
            used = &medium_priority_used;
        }
        break;
    default:
        safety::assert_or_safety_procedure(false, "PacketPayload::add: Invalid priority");
        return false;
    }

    if (destination == nullptr) {
        if (dropped_record_count < std::numeric_limits<uint32_t>::max()) {
            dropped_record_count++;
        }
        return false;
    }

    memcpy(destination + *used, data, data->size);
    *used += data->size;
    if (destination == high_priority_buf) {
        high_priority_count++;
    } else {
        medium_priority_count++;
    }
    return true;
}


void PacketPayload::clear_queues() {
    high_priority_used = 0;
    high_priority_count = 0;
    medium_priority_used = 0;
    medium_priority_count = 0;
}

uint16_t PacketPayload::get_high_priority_queue_size() const {
    return high_priority_count;
}

uint16_t PacketPayload::get_medium_priority_queue_size() const {
    return medium_priority_count;
}

uint32_t PacketPayload::get_dropped_record_count() const {
    return dropped_record_count;
}

uint16_t PacketPayload::get_max_size() const {
    return max_data_size;
}


void PacketPayload::place_incoming_data_in_mega_struct(CommsData* data) {
    HiveData& hive_data = comms_layer.get_hive_data();
    
    // Serial.printf("Placing incoming in mega struct: %s\n", to_string(data->type_label).c_str());
    hive_data.set_data(data);
}

}   // namespace Comms
