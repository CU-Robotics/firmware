#pragma once

#include <cassert>                              // for assert

#include "comms/data/comms_data.hpp"            // for CommsData
#include <Arduino.h>                            // for Serial


namespace Comms {

/// @brief Constructs and stores the data part of a packet we want to send over a physical layer.
class PacketPayload {
public:
    /// @brief Construct a packet payload over caller-owned staging buffers.
    /// @param high_priority_buffer Buffer used to stage high-priority records.
    /// @param medium_priority_buffer Buffer used to stage medium-priority records.
    /// @param max_data_size Capacity of each staging buffer and the output payload.
    PacketPayload(uint8_t* high_priority_buffer, uint8_t* medium_priority_buffer, uint16_t max_data_size);

public:
    /// @brief Add a complete CommsData record to the correct priority staging buffer.
    /// @param data The CommsData record to copy synchronously.
    /// @return True if the complete record was staged, false if capacity was exhausted.
    bool add(const CommsData* data);

    /// @brief Construct a payload from complete staged records in priority and FIFO order.
    /// @param destination Caller-owned output buffer with get_max_size() bytes available.
    /// @return Number of complete record bytes written, excluding any sentinel.
    uint16_t construct_data(uint8_t* destination);

    /// @brief Deconstructs the data packet. Places each CommsData into the correct place in the mega structs  
    /// @param data The raw data buffer.
    /// @param size The size of the raw data buffer.
    /// @note This is thread safe
    void deconstruct_data(uint8_t* data, uint16_t size);


    /// @brief Clear all staged records.
    void clear_queues();

    /// @brief Get the size of the high priority send queue.
    /// @return The size of the high priority send queue.
    uint16_t get_high_priority_queue_size() const;

    /// @brief Get the size of the medium priority send queue.
    /// @return The size of the medium priority send queue.
    uint16_t get_medium_priority_queue_size() const;

    /// @brief Get the cumulative count of valid records rejected for capacity.
    /// @return The saturating count of dropped records.
    uint32_t get_dropped_record_count() const;
    
    /// @brief Get the output and per-priority staging capacity.
    /// @return The capacity in bytes.
    uint16_t get_max_size() const;
    

private:
    /// @brief Pack a complete FIFO prefix from one staging buffer.
    /// @param source Staging buffer to scan and compact.
    /// @param used Number of staged bytes; updated after packing.
    /// @param count Number of staged records; updated after packing.
    /// @param destination Output position for the packed prefix.
    /// @param destination_capacity Remaining output capacity.
    /// @return Number of complete record bytes written.
    uint16_t pack_staged_buffer(uint8_t* source, uint16_t& used, uint16_t& count, uint8_t* destination, uint16_t destination_capacity);

    /// @brief Place the incoming data in the mega struct.
    /// @param data The CommsData to place in the mega struct.
    void place_incoming_data_in_mega_struct(CommsData* data);

    /// @brief The maximum size of each staging buffer and output payload.
    uint16_t max_data_size = 0;

    /// @brief Caller-owned buffer storing staged high-priority data.
    uint8_t* high_priority_buf = nullptr;
    /// @brief Number of bytes currently staged in the high-priority buffer.
    uint16_t high_priority_used = 0;
    /// @brief Count of high-priority records currently staged.
    uint16_t high_priority_count = 0;

    /// @brief Caller-owned buffer storing staged medium-priority data.
    uint8_t* medium_priority_buf = nullptr;
    /// @brief Number of bytes currently staged in the medium-priority buffer.
    uint16_t medium_priority_used = 0;
    /// @brief Count of medium-priority records currently staged.
    uint16_t medium_priority_count = 0;

    /// @brief Saturating count of valid records rejected because staging was full.
    uint32_t dropped_record_count = 0;
};

}   // namespace Comms
