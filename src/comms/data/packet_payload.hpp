#pragma once

#include <queue>
#include <string.h>

#include "comms_data.hpp"

namespace Comms {

/// @brief Constructs and stores the data part of a packet we want to send over a physical layer.
class PacketPayload {
public:
    /// @brief Default constructor for PacketPayload
    PacketPayload() = default;

    /// @brief Constructs PacketPayload by allocating raw_data to store packet data, and configuring a max size for a data type.
    /// @param max_data_size Maximum size of the data we can store in a packet
    PacketPayload(uint16_t max_data_size);

    /// @brief Destructor for PacketPayload, deallocates memory used by raw_data buffer.
    ~PacketPayload();

    /// @brief Assembles the data in queues to create a sendable sequence over a PhysicalMedium.
    /// @note The structure of this payload is as follows: a CommsData derived object is inserted, with the component
    /// @note identifiable as CommsData appearing first. This data can be read to retrieve a TypeLabel and a size,
    /// @note which can be used to navigate to the next element in the payload, as well as copy the data out into
    /// @note a particular type. This is placed into a packet, which gets deconstructed as part of the CommsLayer::decode() routine.
    void construct_data();

    // void deconstruct_data();    // TODO

    /// @brief Adds a pre-configured piece of data to the PacketPayload queue, to be assembled into a packet later.
    /// @param data Pointer to the data that we are appending.
    /// @note This function will automatically free the data pointed to, assuming it is heap-allocated.
    /// @note This function should, in general, only be called by the send() method of the Container template class.
    /// @note *data pointer MUST be heap allocated, otherwise you WILL cause invalid free errors!!!!!*
    void add(CommsData* data);

    /// @brief Returns a pointer to the raw constructed data after construct_data() is called.
    /// @return Pointer to the raw byte array
    uint8_t* data();

    /// @brief Returns the size of data currently in use in the raw data buffer.
    /// @return Size of the data in the data buffer
    uint16_t size();
private:
    /// @brief Empties the contents of the raw data buffer (sets all entries to 0)
    void clear();

    /// @brief Attempts to append all of the data in a queue to the raw byte buffer, stopping when the queue is empty or the buffer is full.
    /// @param queue The queue of CommsData pointers to read from
    void append_data_from_queue(std::queue<CommsData*>& queue);

    /// @brief Attempts to append a single piece of data to the raw byte buffer.
    /// @param data A pointer to a single CommsData derived object to append
    /// @return True on success, otherwise false
    bool try_append_data(CommsData* data);

    // void fill_logging_data_from_queue(std::queue <LoggingData*>& logging_queue);

    // bool try_append_splittable_logging_data(LoggingData& log);

    /// @brief Queue of data objects to send with high priority (this is sent until it is empty first)
    std::queue <CommsData*> high_priority_send_queue;
    /// @brief Queue of data objects to send with medium prority (this will only be sent if high priority queue is empty)
    std::queue <CommsData*> medium_priority_send_queue;
    // std::queue <LoggingData*> logging_send_queue;

    /// @brief Remaining size in the data buffer to write to
    /// @note this type means our max packet size is 2^16-1 bytes long.
    uint16_t remaining_size = 0;

    /// @brief Pointer to the raw byte array
    uint8_t* raw_data = nullptr;

    /// @brief Total amount of data we can fit into a single payload.
    uint16_t max_data_size = 0;
};


}