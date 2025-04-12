#pragma once

#include <queue>                                // for std::queue

#if defined(HIVE)
#include <stdexcept>                            // for std::runtime_error
#elif defined(FIRMWARE)
#include <cassert>                              // for assert
#endif

#if defined(HIVE)
#include <doctest/doctest.h>                    // for TEST_CASE
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#include "modules/comms/data/logging_data.hpp"  // for LoggingData
#include "modules/comms/data/firmware_data.hpp" // for FirmwareData
#include "modules/comms/data/hive_data.hpp"     // for HiveData
#include "modules/hive/environment.hpp"         // for Hive
#elif defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/data/logging_data.hpp"          // for LoggingData
#include "comms/data/hive_data.hpp"             // for HiveData
#include "comms/data/firmware_data.hpp"         // for FirmwareData
#include <Arduino.h>                            // for Serial
#endif

namespace Comms {

/// @brief Constructs and stores the data part of a packet we want to send over a physical layer.
class PacketPayload {
public:
    /// @brief The maximum size of the queue.
    constexpr static uint16_t MAX_QUEUE_SIZE = 50;

    /// @brief Default constructor.
    PacketPayload() = default;

    /// @brief Constructor, allocates the raw data buffer.
    /// @param max_data_size The maximum size of the data part of the packet.
    PacketPayload(uint16_t max_data_size);

    /// @brief Destructor, cleans up the raw data buffer.
    ~PacketPayload();

public:
    /// @brief Add a CommsData to the correct queue.
    /// @param data The CommsData to add.
    /// @note This is thread safe
    void add(CommsData* data);

    /// @brief Using the CommsDatas in the queues, construct a complete data packet.
    /// @note Access this output using the data() function.
    /// @note This is thread safe
    void construct_data();

    /// @brief Deconstructs the data packet. Places each CommsData into the correct place in the mega structs  
    /// @param data The raw data buffer.
    /// @param size The size of the raw data buffer.
    /// @note This is thread safe
    void deconstruct_data(uint8_t* data, uint16_t size);

    /// @brief Get the raw data buffer.
    /// @return The raw data buffer.
    /// @note This is a complete data packet.
    /// @note This is only valid after construct_data() has been called.
    /// @note This is not thread safe
    uint8_t* data();

    /// @brief Clear the queues and the raw data buffer.
    /// @note This is thread safe
    void clear_queues();

    /// @brief Get the size of the high priority send queue.
    /// @return The size of the high priority send queue.
    uint16_t get_high_priority_queue_size() const;

    /// @brief Get the size of the medium priority send queue.
    /// @return The size of the medium priority send queue.
    uint16_t get_medium_priority_queue_size() const;
    
    /// @brief Get the size of the logging send queue.
    /// @return The size of the logging send queue.
    uint16_t get_logging_queue_size() const;

private:
    /// @brief Clear the raw data buffer.
    void clear_raw_data();

    /// @brief Append as much data from a queue to the raw data buffer as possible.
    /// @param queue The queue to append data from.
    void append_data_from_queue(std::queue<CommsData*>& queue);

    /// @brief Try to append a CommsData to the raw data buffer.
    /// @param data The CommsData to append.
    /// @return True if the data was appended, false if it could not be appended.
    bool try_append_data(CommsData* data);

    /// @brief Fill the raw data buffer with LoggingData.
    /// @param logging_queue The queue to fill the raw data buffer with.
    /// @note This is a distinct function because LoggingData has a dynamic size AND it can be split across multiple packets.
    void fill_logging_data_from_queue(std::queue<LoggingData*>& logging_queue);

    /// @brief Try to append a LoggingData to the raw data buffer.
    /// @param log The LoggingData to append.
    /// @return True if the data was appended, false if it could not be appended.
    /// @note This is a distinct function because LoggingData has a dynamic size AND it can be split across multiple packets.
    bool try_append_splittable_logging_data(LoggingData* log);

    /// @brief Place the incoming data in the mega struct.
    /// @param data The CommsData to place in the mega struct.
    void place_incoming_data_in_mega_struct(CommsData* data);

    /// @brief Place the outgoing data in the mega struct.
    /// @param data The CommsData to place in the mega struct.
    void place_outgoing_data_in_mega_struct(CommsData* data);

private:
    /// @brief The high priority send queue.
    std::queue<CommsData*> high_priority_send_queue;
    /// @brief The medium priority send queue.
    std::queue<CommsData*> medium_priority_send_queue;
    /// @brief The logging send queue.
    std::queue<LoggingData*> logging_send_queue;

    /// @brief The raw data buffer. This is the complete data packet.
    uint8_t* raw_data = nullptr;
    /// @brief The maximum size of this data packet.
    uint16_t max_data_size = 0;
    /// @brief The remaining size of the data packet.
    uint16_t remaining_data_size = 0;

#if defined(HIVE)
    /// @brief Mutex to protect concurrent accesses to this PacketPayload
    std::mutex m_mutex;
#endif
};

}   // namespace Comms