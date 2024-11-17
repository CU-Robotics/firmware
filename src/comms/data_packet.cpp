#include "data_packet.hpp"

// --- BuffEncoderData Methods ---

void BuffEncoderData::print() {
    Serial.println("Buff Encoder:");
    Serial.print("Angle: ");
    Serial.println(m_angle);
}

// --- ICMSensorData Methods ---

void ICMSensorData::deserialize(const uint8_t* data, size_t& offset) {
    // Deserialize each field
    memcpy(&accel_X, data + offset, sizeof(accel_X));
    offset += sizeof(accel_X);
    memcpy(&accel_Y, data + offset, sizeof(accel_Y));
    offset += sizeof(accel_Y);
    memcpy(&accel_Z, data + offset, sizeof(accel_Z));
    offset += sizeof(accel_Z);
    memcpy(&gyro_X, data + offset, sizeof(gyro_X));
    offset += sizeof(gyro_X);
    memcpy(&gyro_Y, data + offset, sizeof(gyro_Y));
    offset += sizeof(gyro_Y);
    memcpy(&gyro_Z, data + offset, sizeof(gyro_Z));
    offset += sizeof(gyro_Z);
    memcpy(&temperature, data + offset, sizeof(temperature));
    offset += sizeof(temperature);
}

void ICMSensorData::print() {
    // Display the temperature data, measured in Celsius
    Serial.print("\t\tTemperature ");
    Serial.print(temperature);
    Serial.println(" deg C");
    // Display the acceleration data, measured in m/s^2
    Serial.print("\t\tAccel X: ");
    Serial.print(accel_X);
    Serial.print(" \tY: ");
    Serial.print(accel_Y);
    Serial.print(" \tZ: ");
    Serial.print(accel_Z);
    Serial.println(" m/s^2 ");
    // Display gyroscope data, measured in radians/s
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro_X);
    Serial.print(" \tY: ");
    Serial.print(gyro_Y);
    Serial.print(" \tZ: ");
    Serial.print(gyro_Z);
    Serial.println(" radians/s ");
    Serial.println();
}

// --- RevSensorData Methods ---

void RevSensorData::deserialize(const uint8_t* data, size_t& offset) {
    id = data[offset++]; // Deserialize ID
    memcpy(&ticks, data + offset, sizeof(ticks));
    offset += sizeof(ticks);
    memcpy(&radians, data + offset, sizeof(radians));
    offset += sizeof(radians);
}

void RevSensorData::print() {
    Serial.println("Rev Encoder:");
    Serial.print("Ticks: ");
    Serial.println(ticks);
    Serial.print("Radians: ");
    Serial.println(radians);
}

// --- TOFSensorData Methods ---

void TOFSensorData::deserialize(const uint8_t* data, size_t& offset) {
    // Deserialize the sensor ID
    id = data[offset++];

    // Deserialize the distance
    latest_distance = data[offset++];
    latest_distance |= data[offset++] << 8;
}

void TOFSensorData::print() {
    Serial.println("TOF Sensor:");
    Serial.printf("\tDistance: %u mm\n", latest_distance);
}

// --- LidarSensorData Methods ---

void LidarSensorData::deserialize(const uint8_t* data, size_t& offset) {
    id = data[offset++]; // Deserialize ID
    memcpy(&current_packet, data + offset, sizeof(current_packet));
    offset += sizeof(current_packet);
    memcpy(&cal, data + offset, sizeof(cal));
    offset += sizeof(cal);
    memcpy(packets, data + offset, D200_NUM_PACKETS_CACHED * sizeof(LidarDataPacketSI));
    offset += D200_NUM_PACKETS_CACHED * sizeof(LidarDataPacketSI);
}

void LidarSensorData::print_latest_packet() {
    LidarDataPacketSI p = packets[current_packet];
    Serial.println("==D200LD14P PACKET==");
    Serial.print("LiDAR speed: ");
    Serial.println(p.lidar_speed);
    Serial.print("start angle: ");
    Serial.println(p.start_angle);
    Serial.println("measurement data: ...");
    Serial.print("end angle: ");
    Serial.println(p.end_angle);
    Serial.print("timestamp: ");
    Serial.println(p.timestamp);
}

// --- comms_data_packet Methods ---

comms_data_packet::comms_data_packet(const Config* config_data) {
    config = config_data; // Store the configuration data
    buff_sensor_count = config_data->num_sensors[0];
    icm_sensor_count = config_data->num_sensors[1];
    rev_sensor_count = config_data->num_sensors[2];
    tof_sensor_count = config_data->num_sensors[3];
    lidar_sensor_count = config_data->num_sensors[4];

    // Allocate memory for sensor arrays
    buff_sensors = new BuffEncoderData[buff_sensor_count];
    icm_sensors = new ICMSensorData[icm_sensor_count];
    rev_sensors = new RevSensorData[rev_sensor_count];
    tof_sensors = new TOFSensorData[tof_sensor_count];
    lidar_sensors = new LidarSensorData[lidar_sensor_count];

    // Initialize sensor IDs to a default value (e.g., 254)
    for (int i = 0; i < buff_sensor_count; ++i) {
        buff_sensors[i].id = 254;
    }
    for (int i = 0; i < icm_sensor_count; ++i) {
        icm_sensors[i].id = 254;
    }
    for (int i = 0; i < rev_sensor_count; ++i) {
        rev_sensors[i].id = 254;
    }
    for (int i = 0; i < tof_sensor_count; ++i) {
        tof_sensors[i].id = 254;
    }
    for (int i = 0; i < lidar_sensor_count; ++i) {
        lidar_sensors[i].id = 254;
    }
}

comms_data_packet::~comms_data_packet() {
    delete[] buff_sensors;
    delete[] icm_sensors;
    delete[] rev_sensors;
    delete[] tof_sensors;
    delete[] lidar_sensors;
}

RefereeData comms_data_packet::getRefData() const {
    return refData;
}

CANData comms_data_packet::getCanData() const {
    return canData;
}

void comms_data_packet::pack_data_packet(
    uint8_t packetBuffer[BUFFER_SIZE],
    State robotState,
    uint8_t ref_data_raw[180],
    CANData* canDataPtr,
    EstimatorManager& estimatorManager,
    D200LD14P& lidar1,
    D200LD14P& lidar2,
    DR16& dr16
) {
    size_t packetOffset = 0;
    timestamp = micros(); // Get current time

    // Pack the timestamp, state, and number of sensors
    memcpy(packetBuffer + packetOffset, &timestamp, sizeof(timestamp));
    packetOffset += sizeof(timestamp);

    memcpy(packetBuffer + packetOffset, &state, sizeof(state));
    packetOffset += sizeof(state);

    memcpy(packetBuffer + packetOffset, &buff_sensor_count, sizeof(buff_sensor_count));
    packetOffset += sizeof(buff_sensor_count);
    memcpy(packetBuffer + packetOffset, &icm_sensor_count, sizeof(icm_sensor_count));
    packetOffset += sizeof(icm_sensor_count);
    memcpy(packetBuffer + packetOffset, &rev_sensor_count, sizeof(rev_sensor_count));
    packetOffset += sizeof(rev_sensor_count);
    memcpy(packetBuffer + packetOffset, &tof_sensor_count, sizeof(tof_sensor_count));
    packetOffset += sizeof(tof_sensor_count);
    memcpy(packetBuffer + packetOffset, &lidar_sensor_count, sizeof(lidar_sensor_count));
    packetOffset += sizeof(lidar_sensor_count);

    // Create the RefereeData
    RefereeData RefData;
    memcpy(RefData.ref_data_raw, ref_data_raw, sizeof(RefData.ref_data_raw)); // Load referee data
    memcpy(packetBuffer + packetOffset, &RefData, sizeof(RefData));           // Copy the RefData into the buffer
    packetOffset += sizeof(RefData);

    // Copy CAN data into the buffer
    memcpy(packetBuffer + packetOffset, canDataPtr, sizeof(CANData));
    packetOffset += sizeof(CANData);

    //copy dr16 data into the dr16 data struct and then copy that into the packet buffer
    dr16_data.fail_bit = dr16.is_fail();
    dr16_data.is_connected = dr16.is_connected();
    memcpy(dr16_data.input, dr16.get_input(), sizeof(dr16_data.input));
    dr16_data.r_stick_x = dr16.get_r_stick_x();
    dr16_data.r_stick_y = dr16.get_r_stick_y();
    dr16_data.l_stick_x = dr16.get_l_stick_x();
    dr16_data.l_stick_y = dr16.get_l_stick_y();
    dr16_data.wheel = dr16.get_wheel();
    dr16_data.l_switch = dr16.get_l_switch();
    dr16_data.r_switch = dr16.get_r_switch();
    dr16_data.mouse_x = dr16.get_mouse_x();
    dr16_data.mouse_y = dr16.get_mouse_y();
    dr16_data.l_mouse_button = dr16.get_l_mouse_button();
    dr16_data.r_mouse_button = dr16.get_r_mouse_button();
    dr16_data.fail_time = dr16.m_failTime;
    dr16_data.prev_time = dr16.m_prevTime;
    dr16_data.disconnect_time = dr16.m_disctTime;
    dr16_data.is_data_valid = dr16.is_data_valid();

    //just pretend I did this a better way
    dr16_data.keys.w = dr16.keys.w;
    dr16_data.keys.s = dr16.keys.s;
    dr16_data.keys.a = dr16.keys.a;
    dr16_data.keys.d = dr16.keys.d;
    dr16_data.keys.shift = dr16.keys.shift;
    dr16_data.keys.ctrl = dr16.keys.ctrl;
    dr16_data.keys.q = dr16.keys.q;
    dr16_data.keys.e = dr16.keys.e;
    dr16_data.keys.r = dr16.keys.r;
    dr16_data.keys.f = dr16.keys.f;
    dr16_data.keys.g = dr16.keys.g;
    dr16_data.keys.z = dr16.keys.z;
    dr16_data.keys.x = dr16.keys.x;
    dr16_data.keys.c = dr16.keys.c;
    dr16_data.keys.v = dr16.keys.v;
    dr16_data.keys.b = dr16.keys.b;


    memcpy(packetBuffer + packetOffset, &dr16_data, sizeof(dr16_data));
    packetOffset += sizeof(dr16_data);



    // Serialize sensor data
    for (int i = 0; i < buff_sensor_count; i++) {
        estimatorManager.getBuffSensor(i).serialize(packetBuffer, packetOffset);
    }

    for (int i = 0; i < icm_sensor_count; i++) {
        estimatorManager.getICMSensor(i).serialize(packetBuffer, packetOffset);
    }

    for (int i = 0; i < rev_sensor_count; i++) {
        estimatorManager.getRevSensor(i).serialize(packetBuffer, packetOffset);
    }

    for (int i = 0; i < tof_sensor_count; i++) {
        estimatorManager.getTOFSensor(i).serialize(packetBuffer, packetOffset);
    }

    // Serialize LiDAR sensors if they exist
    if (lidar_sensor_count == 2) {
        lidar1.serialize(packetBuffer, packetOffset);
        lidar2.serialize(packetBuffer, packetOffset);
    }
}

void comms_data_packet::unpack_data_packet(uint8_t packetBuffer[BUFFER_SIZE]) {
    size_t packetOffset = 0;

    // Unpack the timestamp, state, and number of sensors
    memcpy(&timestamp, packetBuffer + packetOffset, sizeof(timestamp));
    packetOffset += sizeof(timestamp);

    memcpy(&state, packetBuffer + packetOffset, sizeof(state));
    packetOffset += sizeof(state);

    memcpy(&buff_sensor_count, packetBuffer + packetOffset, sizeof(buff_sensor_count));
    packetOffset += sizeof(buff_sensor_count);
    memcpy(&icm_sensor_count, packetBuffer + packetOffset, sizeof(icm_sensor_count));
    packetOffset += sizeof(icm_sensor_count);
    memcpy(&rev_sensor_count, packetBuffer + packetOffset, sizeof(rev_sensor_count));
    packetOffset += sizeof(rev_sensor_count);
    memcpy(&tof_sensor_count, packetBuffer + packetOffset, sizeof(tof_sensor_count));
    packetOffset += sizeof(tof_sensor_count);
    memcpy(&lidar_sensor_count, packetBuffer + packetOffset, sizeof(lidar_sensor_count));
    packetOffset += sizeof(lidar_sensor_count);

    // Unpack the RefereeData
    memcpy(&refData, packetBuffer + packetOffset, sizeof(refData));
    packetOffset += sizeof(refData);

    // Unpack CAN data
    memcpy(&canData, packetBuffer + packetOffset, sizeof(CANData));
    packetOffset += sizeof(CANData);

    // Unpack DR16 data
    memcpy(&dr16_data, packetBuffer + packetOffset, sizeof(dr16_data));
    packetOffset += sizeof(dr16_data);


    // Unpack Buff Encoders
    for (int i = 0; i < buff_sensor_count; i++) {
        buff_sensors[i].id = packetBuffer[packetOffset++];
        memcpy(&buff_sensors[i].m_angle, packetBuffer + packetOffset, sizeof(buff_sensors[i].m_angle));
        packetOffset += sizeof(buff_sensors[i].m_angle);
    }

    // Unpack ICM Sensors
    for (int i = 0; i < icm_sensor_count; i++) {
        icm_sensors[i].deserialize(packetBuffer, packetOffset);
    }

    // Unpack Rev Encoders
    for (int i = 0; i < rev_sensor_count; i++) {
        rev_sensors[i].deserialize(packetBuffer, packetOffset);
    }

    // Unpack TOF Sensors
    for (int i = 0; i < tof_sensor_count; i++) {
        tof_sensors[i].deserialize(packetBuffer, packetOffset);
    }

    // Unpack LiDAR Sensors
    if (lidar_sensor_count == 2) {
        lidar_sensors[0].deserialize(packetBuffer, packetOffset);
        lidar_sensors[1].deserialize(packetBuffer, packetOffset);
    }
}