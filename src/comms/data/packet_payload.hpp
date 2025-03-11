#pragma once

#include <memory>               // for shared_ptr
#include <queue>                // for queue

#if defined(HIVE)
#include <exception>            // for runtime_error
#elif defined(FIRMWARE)
#include <cassert>              // for assert
#endif

#if defined(HIVE)
#include <doctest/doctest.h>                    // for TEST_CASE
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#include "modules/comms/data/logging_data.hpp"  // for LoggingData
#include "modules/comms/data/firmware_data.hpp" // for FirmwareData
#include "modules/hive/environment.hpp"         // for Hive
#elif defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/data/logging_data.hpp"          // for LoggingData
#include "comms/data/hive_data.hpp"             // for HiveData
#include <Arduino.h>                            // for Serial
#endif

namespace Comms {

/// @brief Constructs and stores the data part of a packet we want to send over a physical layer.
class PacketPayload {
public:
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
    void add(CommsData* data);

    /// @brief Using the CommsDatas in the queues, construct a complete data packet.
    /// @note Access this output using the data() function.
    void construct_data();

    /// @brief Deconstructs the data packet. Places each CommsData into the correct place in the mega structs  
    /// @param data The raw data buffer.
    /// @param size The size of the raw data buffer.
    void deconstruct_data(uint8_t* data, uint16_t size);

    /// @brief Get the raw data buffer.
    /// @return The raw data buffer.
    /// @note This is a complete data packet.
    uint8_t* data();

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

    /// @brief Place the data in the mega struct.
    /// @param data The CommsData to place in the mega struct.
    void place_data_in_mega_struct(CommsData* data);

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

    /// @brief The maximum size of the queue.
    constexpr static uint16_t MAX_QUEUE_SIZE = 50;

};

}   // namespace Comms

#if defined(HIVE)

TEST_CASE("Testing packet payload") {

    // Create a packet payload with a max size of 103 bytes
    // 103 was chosen because it is 3 bytes more than the any multiple size of the test data object
    // 3 is also less then the size of the CommsData header
    constexpr int PACKET_SIZE = 103;
    Comms::PacketPayload payload(PACKET_SIZE);
    Comms::PacketPayload default_payload;   // "tests" the default constructor

    // Create a test data object
    TestData data;
    data.x = 1.5f;
    data.y = 2.5f;
    data.z = 3.5f;
    data.w = 0x12345678;

    SUBCASE("Testing payload fills raw data") {
        payload.add(new TestData(data));
        
        payload.construct_data();

        uint8_t* raw_data = payload.data();
        uint8_t* test_data_ptr = reinterpret_cast<uint8_t*>(&data);

        REQUIRE(memcmp(raw_data, test_data_ptr, sizeof(TestData)) == 0);
    }

    SUBCASE("Testing payload pops data") {
        payload.add(new TestData(data));

        // construct once, the data should be in the raw data buffer
        payload.construct_data();
        // construct again, this clears the buffer, and our previously added object should be popped
        payload.construct_data();

        uint8_t empty_buffer[sizeof(TestData)];
        memset(empty_buffer, 0, sizeof(TestData));

        uint8_t* raw_data = payload.data();

        // ensure the raw data buffer is now empty
        REQUIRE(memcmp(raw_data, empty_buffer, sizeof(TestData)) == 0);
    }

    SUBCASE("Testing fill too much data does not crash") {
        // fill the payload with too much data
        // this adds 1 more than the max size of the payload
        for (size_t i = 0; i < PACKET_SIZE / sizeof(TestData) + 1; i++) {
            payload.add(new TestData(data));
        }

        // construct the data, this should not crash
        payload.construct_data();
    }

    SUBCASE("Testing fill to much data still leaves data in queue") {
        // fill the payload with too much data
        // this adds 1 more than the max size of the payload
        for (size_t i = 0; i < PACKET_SIZE / sizeof(TestData) + 1; i++) {
            payload.add(new TestData(data));
        }

        // construct the data, this should not crash
        payload.construct_data();

        // construct the data again, this should not crash
        payload.construct_data();

        uint8_t* raw_data = payload.data();
        uint8_t empty_buffer[sizeof(TestData)];
        memset(empty_buffer, 0, sizeof(TestData));

        // ensure the raw data buffer is not empty
        REQUIRE(memcmp(raw_data, empty_buffer, sizeof(TestData)) != 0);
    }

    SUBCASE("Testing different priority data") {
        TestData high_priority_data;
        high_priority_data.x = 1.5f;
        high_priority_data.y = 2.5f;
        high_priority_data.z = 3.5f;
        high_priority_data.w = 0x12345678;

        TestData medium_priority_data;
        medium_priority_data.priority = Comms::Priority::Medium;
        medium_priority_data.x = 4.5f;
        medium_priority_data.y = 5.5f;
        medium_priority_data.z = 6.5f;
        medium_priority_data.w = 0x87654321;
        
        Comms::LoggingData logging_data;
        char log_buffer[] = "This is a test log message.";
        logging_data.deserialize(log_buffer, sizeof(log_buffer));

        payload.add(new TestData(medium_priority_data));
        payload.add(new TestData(high_priority_data));
        payload.add(new Comms::LoggingData(logging_data));

        payload.construct_data();

        uint8_t* raw_data = payload.data();

        // ensure the high priority data is in the buffer first
        REQUIRE(memcmp(raw_data, &high_priority_data, sizeof(TestData)) == 0);

        // ensure the medium priority data is in the buffer next
        raw_data += sizeof(TestData);
        REQUIRE(memcmp(raw_data, &medium_priority_data, sizeof(TestData)) == 0);

        // ensure the logging data is in the buffer last
        raw_data += sizeof(TestData) + sizeof(Comms::CommsData);
        Comms::LoggingData logging_data_out;
        logging_data_out.deserialize((char*)raw_data, sizeof(log_buffer));
        REQUIRE(logging_data_out.get_logs() == logging_data.get_logs());
    }

    SUBCASE("Testing adding invalid priority data") {
        TestData invalid_priority_data;
        invalid_priority_data.priority = static_cast<Comms::Priority>(15);

        REQUIRE_THROWS_AS(payload.add(new TestData(invalid_priority_data)), std::runtime_error);
    }

    SUBCASE("Testing adding logging data with not enough space available") {
        Comms::LoggingData logging_data;
        char log_buffer[] = "This is a test log message.";
        logging_data.deserialize(log_buffer, sizeof(log_buffer));

        // this adds the most payloads as possible, leaves a little bit of room left
        for (size_t i = 0; i < PACKET_SIZE / sizeof(TestData); i++) {
            payload.add(new TestData(data));
        }

        // add the logging data
        payload.add(new Comms::LoggingData(logging_data));

        // construct the data, this should not crash
        payload.construct_data();

        // construct the data again, this should not crash
        payload.construct_data();

        uint8_t* raw_data = payload.data();
        uint8_t empty_buffer[sizeof(TestData)];
        memset(empty_buffer, 0, sizeof(TestData));

        // ensure the raw data buffer is not empty
        REQUIRE(memcmp(raw_data, empty_buffer, sizeof(TestData)) != 0);

        // ensure the logging data is in the buffer
        Comms::LoggingData logging_data_out;
        raw_data += sizeof(Comms::CommsData);
        logging_data_out.deserialize((char*)raw_data, sizeof(log_buffer));
        REQUIRE(logging_data_out.get_logs() == logging_data.get_logs());
    }

    SUBCASE("Testing adding partial logging data") {
        Comms::LoggingData logging_data;
        char log_buffer[] = "This is a test log message.";
        logging_data.deserialize(log_buffer, sizeof(log_buffer));

        // fill the payload with 1 less than the max size of the payload
        for (size_t i = 0; i < PACKET_SIZE / sizeof(TestData) - 1; i++) {
            payload.add(new TestData(data));
        }

        int test_data_added = (PACKET_SIZE / sizeof(TestData) - 1);

        // add the logging data
        payload.add(new Comms::LoggingData(logging_data));

        // construct the data, this should not crash
        payload.construct_data();

        // verify there are test_data_added instances of the test data in the buffer
        uint8_t* raw_data = payload.data();
        uint8_t* test_data_ptr = reinterpret_cast<uint8_t*>(&data);
        for (int i = 0; i < test_data_added; i++) {
            REQUIRE(memcmp(raw_data, test_data_ptr, sizeof(TestData)) == 0);
            raw_data += sizeof(TestData);
        }

        // ensure there some logging data in the buffer
        Comms::LoggingData logging_data_out1;
        raw_data += sizeof(Comms::CommsData);
        logging_data_out1.deserialize((char*)raw_data, sizeof(log_buffer));
        // this part of the logging data should not be empty
        REQUIRE(logging_data_out1.get_logs() != "");

        // construct the data again, this should not crash
        payload.construct_data();

        // verify that the rest of the logging data is in the buffer
        raw_data = payload.data();

        Comms::LoggingData logging_data_out2;
        raw_data += sizeof(Comms::CommsData);
        logging_data_out2.deserialize((char*)raw_data, sizeof(log_buffer));
        // this part of the logging data should not be empty
        REQUIRE(logging_data_out2.get_logs() != "");

        // ensure that all the logging data was eventually grabbed
        REQUIRE(logging_data_out1.get_logs() + logging_data_out2.get_logs() == logging_data.get_logs());

        // construct again to ensure the queues are empty
        payload.construct_data();

        // ensure the data is empty
        raw_data = payload.data();
        uint8_t empty_buffer[sizeof(TestData)];
        memset(empty_buffer, 0, sizeof(TestData));

        // ensure the raw data buffer is not empty
        REQUIRE(memcmp(raw_data, empty_buffer, sizeof(TestData)) == 0);
    }

    SUBCASE("Testing deconstruct data") {
        // initialize FirmwareData in hive environment
        Comms::FirmwareData* firmware_data = new Comms::FirmwareData();
        Hive::env->firmware_data = firmware_data;

        TestData data1;
        data1.x = 1.5f;
        data1.y = 2.5f;
        data1.z = 3.5f;
        data1.w = 0x12345678;

        payload.add(new TestData(data1));

        payload.construct_data();

        payload.deconstruct_data(payload.data(), PACKET_SIZE);

        // ensure the data was placed in the correct place in the mega struct
        REQUIRE(Hive::env->firmware_data->test_data.x == data1.x);
        REQUIRE(Hive::env->firmware_data->test_data.y == data1.y);
        REQUIRE(Hive::env->firmware_data->test_data.z == data1.z);
        REQUIRE(Hive::env->firmware_data->test_data.w == data1.w);
    }

    SUBCASE("Testing deconstructing full packet") {
        // initialize FirmwareData in hive environment
        Comms::FirmwareData* firmware_data = new Comms::FirmwareData();
        Hive::env->firmware_data = firmware_data;

        TestData data;
        data.x = 1.5f;
        data.y = 2.5f;
        data.z = 3.5f;
        data.w = 0x12345678;

        Comms::LoggingData logging_data;
        char log_buffer[] = "This is a very long log message and it probably will be split into multiple packets. Blah";
        logging_data.deserialize(log_buffer, sizeof(log_buffer));

        payload.add(new TestData(data));
        payload.add(new Comms::LoggingData(logging_data));

        payload.construct_data();

        payload.deconstruct_data(payload.data(), PACKET_SIZE);

        // ensure the data was placed in the correct place in the mega struct
        REQUIRE(Hive::env->firmware_data->test_data.x == data.x);
        REQUIRE(Hive::env->firmware_data->test_data.y == data.y);
        REQUIRE(Hive::env->firmware_data->test_data.z == data.z);
        REQUIRE(Hive::env->firmware_data->test_data.w == data.w);

        // store the firmware logging data
        Comms::LoggingData log_data1;
        memcpy(&log_data1, &Hive::env->firmware_data->logging_data, sizeof(Comms::LoggingData));

        // construct and deconstruct again to get the rest of the logging data
        payload.construct_data();

        payload.deconstruct_data(payload.data(), PACKET_SIZE);

        // this data should not have changed
        REQUIRE(Hive::env->firmware_data->test_data.x == data.x);
        REQUIRE(Hive::env->firmware_data->test_data.y == data.y);
        REQUIRE(Hive::env->firmware_data->test_data.z == data.z);
        REQUIRE(Hive::env->firmware_data->test_data.w == data.w);

        REQUIRE(log_data1.get_logs() + Hive::env->firmware_data->logging_data.get_logs() == logging_data.get_logs());
    }

    SUBCASE("Testing bad type label deconstruct") {
        // initialize FirmwareData in hive environment
        Comms::FirmwareData* firmware_data = new Comms::FirmwareData();
        Hive::env->firmware_data = firmware_data;

        TestData data;
        data.x = 1.5f;
        data.y = 2.5f;
        data.z = 3.5f;
        data.w = 0x12345678;

        // set data to have a bad type label
        data.type_label = static_cast<Comms::TypeLabel>(15);

        // add and construct
        payload.add(new TestData(data));

        payload.construct_data();

        // deconstruct should throw an error
        REQUIRE_THROWS_AS(payload.deconstruct_data(payload.data(), PACKET_SIZE), std::runtime_error);
    }

    SUBCASE("Testing destructor") {
        Comms::PacketPayload payload(PACKET_SIZE);

        TestData data;
        data.priority = Comms::Priority::High;
        payload.add(new TestData(data));

        data.priority = Comms::Priority::Medium;
        payload.add(new TestData(data));

        data.priority = Comms::Priority::Logging;
        payload.add(new TestData(data));
    }
}

#endif  // HIVE