#include "packet_payload.hpp"

#include <algorithm>    // for min
#include <mutex>        // for std::lock_guard, std::mutex
#include <string.h>     // for memset/memcpy
#include <assert.h>     // for assert

#if defined(HIVE)
#include <iostream>     // for std::cout
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
        place_data_in_mega_struct(header);


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

void PacketPayload::place_data_in_mega_struct(CommsData* data) {
#if defined(HIVE)

    std::lock_guard<std::mutex> lock(Hive::env->firmware_data_mutex);
    std::cout << "Placing data in mega struct: " << to_string(data->type_label) << std::endl;
    switch (data->type_label) {
    case TypeLabel::TestData: {
        // place the data in the mega struct
        TestData* test_data = static_cast<TestData*>(data);
        memcpy(&Hive::env->firmware_data->test_data, test_data, sizeof(TestData));
        break;
    }
    case TypeLabel::LoggingData: {
        // place the data in the mega struct
        LoggingData* logging_data = static_cast<LoggingData*>(data);
        memcpy(&Hive::env->firmware_data->logging_data, logging_data, sizeof(LoggingData));
        std::cout << logging_data->get_logs() << std::endl;
        break;
    }
    case TypeLabel::TempRobotState: {
        // place the data in the mega struct
        TempRobotState* temp_robot_state = static_cast<TempRobotState*>(data);
        memcpy(&Hive::env->firmware_data->temp_robot_state, temp_robot_state, sizeof(TempRobotState));
        break;
    }
    case TypeLabel::EstimatedState: {
        // place the data in the mega struct
        EstimatedState* estimated_state = static_cast<EstimatedState*>(data);
        memcpy(&Hive::env->firmware_data->estimated_state, estimated_state, sizeof(EstimatedState));
        break;
    }
    case TypeLabel::DR16Data: {
        // place the data in the mega struct
        DR16Data* dr16_data = static_cast<DR16Data*>(data);
        memcpy(&Hive::env->firmware_data->dr16_data, dr16_data, sizeof(DR16Data));
        break;
    }
    case TypeLabel::BuffEncoderData: {
        //determine if the data is for yaw or pitch
        BuffEncoderData* buff_encoder_data = static_cast<BuffEncoderData*>(data);
        if (buff_encoder_data->id == 0) {
            memcpy(&Hive::env->firmware_data->yaw_buff_encoder, buff_encoder_data, sizeof(BuffEncoderData));
        } else if (buff_encoder_data->id == 1) {
            memcpy(&Hive::env->firmware_data->pitch_buff_encoder, buff_encoder_data, sizeof(BuffEncoderData));
        }
        break;
    }
    case TypeLabel::RevEncoderData: {
        //determine which rev encoder the data is for
        RevSensorData* rev_encoder_data = static_cast<RevSensorData*>(data);
        if (rev_encoder_data->id == 0) {
            memcpy(&Hive::env->firmware_data->rev_sensor_0, rev_encoder_data, sizeof(RevSensorData));
        } else if (rev_encoder_data->id == 1) {
            memcpy(&Hive::env->firmware_data->rev_sensor_1, rev_encoder_data, sizeof(RevSensorData));
        } else if (rev_encoder_data->id == 2) {
            memcpy(&Hive::env->firmware_data->rev_sensor_2, rev_encoder_data, sizeof(RevSensorData));
        }
        break;
    }
    case TypeLabel::ICMSensorData: {
        // place the data in the mega struct
        ICMSensorData* icm_sensor_data = static_cast<ICMSensorData*>(data);
        memcpy(&Hive::env->firmware_data->icm_sensor, icm_sensor_data, sizeof(ICMSensorData));
        break;
    }
    case TypeLabel::TOFSensorData: {
        // place the data in the mega struct
        TOFSensorData* tof_sensor_data = static_cast<TOFSensorData*>(data);
        memcpy(&Hive::env->firmware_data->tof_sensor, tof_sensor_data, sizeof(TOFSensorData));
        break;
    }
    case TypeLabel::LidarSensorData: {
        //determine which lidar sensor the data is for
        LidarSensorData* lidar_sensor_data = static_cast<LidarSensorData*>(data);
        if (lidar_sensor_data->id == 0) {
            memcpy(&Hive::env->firmware_data->lidar_sensor_0, lidar_sensor_data, sizeof(LidarSensorData));
        } else if (lidar_sensor_data->id == 1) {
            memcpy(&Hive::env->firmware_data->lidar_sensor_1, lidar_sensor_data, sizeof(LidarSensorData));
        }
        break;
    }
    default:
        throw std::runtime_error("Invalid type label given to place in mega struct: " + std::to_string(static_cast<uint8_t>(data->type_label)));
    }
    
#elif defined(FIRMWARE)
    
    Serial.printf("Placing data in mega struct: %s\n", to_string(data->type_label).c_str());
    switch (data->type_label) {
    case TypeLabel::TestData: {
        // place the data in the mega struct
        TestData* test_data = static_cast<TestData*>(data);
        memcpy(&hive_data.test_data, test_data, sizeof(TestData));
        break;
    }
    case TypeLabel::TargetState: {
        // place the data in the mega struct
        TargetState* target_state = static_cast<TargetState*>(data);
        memcpy(&hive_data.target_state, target_state, sizeof(TargetState));
        break;
    }
    case TypeLabel::OverrideState: {
        // place the data in the mega struct
        OverrideState* override_state = static_cast<OverrideState*>(data);
        memcpy(&hive_data.override_state, override_state, sizeof(OverrideState));
        break;
    }
    default:
        assert(false && "Invalid type label given to place in mega struct");
    }

#endif
}

}   // namespace Comms