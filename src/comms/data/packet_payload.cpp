#include "packet_payload.hpp"

#include <algorithm>    // for min
#include <mutex>        // for std::lock_guard, std::mutex
#include <string.h>     // for memset/memcpy
#include <assert.h>     // for assert

#if defined(HIVE)
#include <iostream>                         // for std::cout
#include "modules/comms/comms_layer.hpp"    // for CommsLayer
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

    while (!logging_send_queue.empty()) {
        delete logging_send_queue.front();
        logging_send_queue.pop();
    }
}

void PacketPayload::construct_data() {
#if defined(HIVE)
    std::lock_guard<std::mutex> lock(m_mutex);
#endif
    
    clear_raw_data();

    append_data_from_queue(high_priority_send_queue);
    append_data_from_queue(medium_priority_send_queue);

    fill_logging_data_from_queue(logging_send_queue);
}

void PacketPayload::deconstruct_data(uint8_t* data, uint16_t size) {
#if defined(HIVE)
    std::lock_guard<std::mutex> lock(m_mutex);
#endif

    // ensure the data is the correct size
    assert(size == max_data_size);

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

void PacketPayload::add(CommsData* data) {
#if defined(HIVE)
    std::lock_guard<std::mutex> lock(m_mutex);
#endif

    switch (data->priority) {
    case Priority::High: {
        if (high_priority_send_queue.size() <= MAX_QUEUE_SIZE) {
            high_priority_send_queue.push(data);
        } else {
            delete data;
        }
        break;
    } case Priority::Medium: {
        if (medium_priority_send_queue.size() <= MAX_QUEUE_SIZE) {
            medium_priority_send_queue.push(data);
        } else {
            delete data;
        }
        break;
    } case Priority::Logging: {
        // a logging priority item must be of type logging data since we grab data from logs dynamically (handled in this file)
        LoggingData* logging = static_cast<LoggingData*>(data);
        if (logging_send_queue.size() <= MAX_QUEUE_SIZE) {
            logging_send_queue.push(logging);
        } else {
            delete data;
        }
        break;
    }
    default:
    #if defined(HIVE)
        throw std::runtime_error("Invalid priority given to add");
    #elif defined(FIRMWARE)
        assert(false && "Invalid priority given to add");
    #endif
    }
}

uint8_t* PacketPayload::data() {
    return raw_data;
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

void PacketPayload::fill_logging_data_from_queue(std::queue<LoggingData*>& queue) {
    while (!queue.empty() && remaining_data_size > 0) {
        LoggingData* next_data = queue.front();

        bool finished_append = try_append_splittable_logging_data(next_data);

        if (finished_append) {
            // TODO: set the data in the mega struct
            // place_outgoing_data_in_mega_struct(next_data);
            
            // if we have appended the entirety of the splittlable comms data
            queue.pop(); // remove the appended item from the queue, going to the next.
        } else {
            // If we could append the full amount (out of space), give up.
            break;
        }
    }
}

bool PacketPayload::try_append_splittable_logging_data(LoggingData* log) {
    // if an empty (except for Header) CommsData doesn't even fit
    if (sizeof(CommsData) > remaining_data_size) {
        return false; // fail
    }

    // where to start the append
    uint16_t append_offset = max_data_size - remaining_data_size;

    // account for the header
    remaining_data_size -= sizeof(CommsData);

    // we want to fill as much as possible:
    // either the rest of the requested data, or the entirety of the remaining size
    uint16_t fill_size = std::min(remaining_data_size, log->remaining_size_to_serialize());

    // append header
    CommsData header = CommsData(log->type_label, log->physical_medium, log->priority, fill_size + sizeof(CommsData));
    memcpy(raw_data + append_offset, &header, sizeof(CommsData));
    append_offset += sizeof(header);

    // append into raw buffer using log's serialize (this is why had to downcast)
    log->serialize(raw_data + append_offset, fill_size);
    remaining_data_size -= fill_size;

    if (log->all_data_has_been_serialized()) {
        // we filled up the rest of the requested logs, we can say we finished
        return true;
    }

    // we may have successfully appended some data,
    // but there is still more data in this LoggingData, so we don't want to say to pop it yet.
    return false;
}

void PacketPayload::place_incoming_data_in_mega_struct(CommsData* data) {
#if defined(HIVE)
    FirmwareData firmware_data = Hive::env->comms_layer->get_firmware_data();

    //std::cout << "Placing data in mega struct: " << to_string(data->type_label) << std::endl;
    firmware_data.set_data(data);

    Hive::env->comms_layer->set_firmware_data(firmware_data);
    
#elif defined(FIRMWARE)

    HiveData& hive_data = comms_layer.get_hive_data();
    
    Serial.printf("Placing incoming in mega struct: %s\n", to_string(data->type_label).c_str());
    hive_data.set_data(data);
#endif
}

void PacketPayload::place_outgoing_data_in_mega_struct(CommsData* data) {
#if defined(HIVE)
    HiveData hive_data = Hive::env->comms_layer->get_hive_data();

    hive_data.set_data(data);

    Hive::env->comms_layer->set_hive_data(hive_data);
#elif defined(FIRMWARE)
    FirmwareData& firmware_data = comms_layer.get_firmware_data();

    Serial.printf("Placing outgoing in mega struct: %s\n", to_string(data->type_label).c_str());
    firmware_data.set_data(data);
#endif
}

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

    // create a CommsLayer object to test the mega struct
    Comms::CommsLayer comms_layer;
    comms_layer.init_without_physical_layers();

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
        TestData data1;
        data1.x = 1.5f;
        data1.y = 2.5f;
        data1.z = 3.5f;
        data1.w = 0x12345678;

        payload.add(new TestData(data1));

        payload.construct_data();

        payload.deconstruct_data(payload.data(), PACKET_SIZE);

        // ensure the data was placed in the correct place in the mega struct
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.x == data1.x);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.y == data1.y);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.z == data1.z);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.w == data1.w);
    }

    SUBCASE("Testing deconstructing full packet") {
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
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.x == data.x);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.y == data.y);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.z == data.z);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.w == data.w);

        // store the firmware logging data
        Comms::LoggingData log_data1 = Hive::env->comms_layer->get_firmware_data().logging_data;

        // construct and deconstruct again to get the rest of the logging data
        payload.construct_data();

        payload.deconstruct_data(payload.data(), PACKET_SIZE);

        // this data should not have changed
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.x == data.x);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.y == data.y);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.z == data.z);
        REQUIRE(Hive::env->comms_layer->get_firmware_data().test_data.w == data.w);

        REQUIRE(log_data1.get_logs() + Hive::env->comms_layer->get_firmware_data().logging_data.get_logs() == logging_data.get_logs());
    }

    SUBCASE("Testing bad type label deconstruct") {
        TestData data;
        data.x = 1.5f;
        data.y = 2.5f;
        data.z = 3.5f;
        data.w = 0x12345678;

        // set data to have a bad type label
        data.type_label = static_cast<Comms::TypeLabel>(15);

        // add and construct
        payload.add(new TestData(data));

        // construct should throw an error
        REQUIRE_THROWS_AS(payload.construct_data(), std::runtime_error);

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

#endif // defined(HIVE)
