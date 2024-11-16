#include <cstdint>
#include <state.hpp>
#include <vector>
#include <rm_can.hpp>
#include "Sensor.hpp"
#include "constants.hpp"
#include "estimator_manager.hpp"
#include "config_layer.hpp"
#include "d200.hpp" 

// Structure to hold referee data
struct RefereeData {
    uint8_t ref_data_raw[180] = { 0 }; // Raw data array for the referee data packet
};

// Structure for the buff encoder sensor
struct buff_sensor {
    uint8_t id;     // Sensor ID
    float m_angle;  // Measured angle

    /// @brief Function to print the sensor data
    void print() {
        Serial.println("Buff Encoder:");
        Serial.print("Angle: ");
        Serial.println(m_angle);
    }
};

// Structure for the ICM sensor
struct icm_sensor {
    uint8_t id;         // Sensor ID
    float accel_X;      // Acceleration in X-axis
    float accel_Y;      // Acceleration in Y-axis
    float accel_Z;      // Acceleration in Z-axis
    float gyro_X;       // Gyroscope reading in X-axis
    float gyro_Y;       // Gyroscope reading in Y-axis
    float gyro_Z;       // Gyroscope reading in Z-axis
    float temperature;  // Temperature reading

    /// @brief Function to deserialize data from a byte array
    /// @param data Data to deserialize
    /// @param offset Offset to update as data is read
    void deserialize(const uint8_t* data, size_t& offset) {
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

    /// @brief Function to print the sensor data
    void print() {
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
};

// Structure for the Rev encoder sensor
struct rev_sensor {
    uint8_t id;      // Sensor ID
    int ticks;       // Encoder ticks
    float radians;   // Angle in radians

    /// @brief Function to deserialize data from a byte array
    /// @param data Data to deserialize
    /// @param offset Offset to update as data is read
    void deserialize(const uint8_t* data, size_t& offset) {
        id = data[offset++]; // Deserialize ID
        memcpy(&ticks, data + offset, sizeof(ticks));
        offset += sizeof(ticks);
        memcpy(&radians, data + offset, sizeof(radians));
        offset += sizeof(radians);
    }

    /// @brief Function to print the sensor data
    void print() {
        Serial.println("Rev Encoder:");
        Serial.print("Ticks: ");
        Serial.println(ticks);
        Serial.print("Radians: ");
        Serial.println(radians);
    }
};

// Structure for the TOF (Time-of-Flight) sensor
struct tof_sensor {
    uint8_t id;                 // Sensor ID
    uint16_t latest_distance;   // Latest distance measurement

    /// @brief Function to deserialize the TOF sensor data
    /// @param data Data to deserialize
    /// @param offset Offset to update as data is read
    void deserialize(const uint8_t* data, size_t& offset) {
        // Deserialize the sensor ID
        id = data[offset++];

        // Deserialize the distance
        latest_distance = data[offset++];
        latest_distance |= data[offset++] << 8;
    }

    /// @brief Function to print the sensor data
    void print() {
        Serial.println("TOF Sensor:");
        Serial.printf("\tDistance: %u mm\n", latest_distance);
    }
};

// Structure for the LiDAR sensor
struct lidar_sensor {
    uint8_t id;                            // Sensor ID
    int current_packet;                    // Index of the current data packet
    D200Calibration cal;                   // Calibration data
    LidarDataPacketSI packets[D200_NUM_PACKETS_CACHED]; // Array of cached data packets

    /// @brief Function to deserialize data from a byte array
    /// @param data Data to deserialize
    /// @param offset Offset to update as data is read
    void deserialize(const uint8_t* data, size_t& offset) {
        id = data[offset++]; // Deserialize ID
        memcpy(&current_packet, data + offset, sizeof(current_packet));
        offset += sizeof(current_packet);
        memcpy(&cal, data + offset, sizeof(cal));
        offset += sizeof(cal);
        memcpy(packets, data + offset, D200_NUM_PACKETS_CACHED * sizeof(LidarDataPacketSI));
        offset += D200_NUM_PACKETS_CACHED * sizeof(LidarDataPacketSI);
    }

    /// @brief Function to print the latest LiDAR data packet
    void print_latest_packet() {
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
};

// Structure for the data packet containing all sensor data and state information
struct data_packet {
    const Config* config; // Pointer to the Config struct storing all configuration data

    uint32_t timestamp; // Current time in microseconds
    State state;        // Robot state

    RefereeData refData; // Referee data
    CANData canData;     // CAN bus data

    int buff_sensor_count;   // Number of buff sensors
    int icm_sensor_count;    // Number of ICM sensors
    int rev_sensor_count;    // Number of Rev sensors
    int tof_sensor_count;    // Number of TOF sensors
    int lidar_sensor_count;  // Number of LiDAR sensors

    buff_sensor* buff_sensors;     // Array of buff sensors
    icm_sensor* icm_sensors;       // Array of ICM sensors
    rev_sensor* rev_sensors;       // Array of Rev sensors
    tof_sensor* tof_sensors;       // Array of TOF sensors
    lidar_sensor* lidar_sensors;   // Array of LiDAR sensors

    /// @brief Getter for RefereeData
    /// @return RefereeData object
    RefereeData getRefData() const { return refData; }

    /// @brief Getter for CANData
    /// @return CANData object
    CANData getCanData() const { return canData; }

    /// @brief Constructor to initialize the data_packet with configuration data
    /// @param config_data Pointer to the configuration data
    data_packet(const Config* config_data) {
        config = config_data; // Store the configuration data
        buff_sensor_count = config_data->num_sensors[0];
        icm_sensor_count = config_data->num_sensors[1];
        rev_sensor_count = config_data->num_sensors[2];
        tof_sensor_count = config_data->num_sensors[3];
        lidar_sensor_count = config_data->num_sensors[4];

        // Allocate memory for sensor arrays
        buff_sensors = new buff_sensor[buff_sensor_count];
        icm_sensors = new icm_sensor[icm_sensor_count];
        rev_sensors = new rev_sensor[rev_sensor_count];
        tof_sensors = new tof_sensor[tof_sensor_count];
        lidar_sensors = new lidar_sensor[lidar_sensor_count];

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

    /// @brief Destructor to clean up allocated memory
    ~data_packet() {
        delete[ ] buff_sensors;
        delete[ ] icm_sensors;
        delete[ ] rev_sensors;
        delete[ ] tof_sensors;
        delete[ ] lidar_sensors;
    }

    /// @brief Function to pack data into a packet buffer
    /// @param packetBuffer Buffer to pack the data into
    /// @param robotState The current state of the robot
    /// @param ref_data_raw Raw referee data
    /// @param canData Pointer to CANData
    /// @param estimatorManager Reference to the EstimatorManager
    /// @param lidar1 Reference to the first LiDAR sensor
    /// @param lidar2 Reference to the second LiDAR sensor
    void packDataPacket(uint8_t packetBuffer[BUFFER_SIZE], State robotState, uint8_t ref_data_raw[180], CANData* canData, EstimatorManager& estimatorManager, D200LD14P& lidar1, D200LD14P& lidar2) {
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

        // // Print packet information for debugging
        // Serial.println("Packing data packet:");
        // Serial.print("Timestamp: ");
        // Serial.println(timestamp);
        // Serial.print("Buff Sensor Count: ");
        // Serial.println(buff_sensor_count);
        // Serial.print("ICM Sensor Count: ");
        // Serial.println(icm_sensor_count);
        // Serial.print("Rev Sensor Count: ");
        // Serial.println(rev_sensor_count);
        // Serial.print("TOF Sensor Count: ");
        // Serial.println(tof_sensor_count);
        // Serial.print("Lidar Sensor Count: ");
        // Serial.println(lidar_sensor_count);

        // Create the RefereeData
        RefereeData RefData;
        memcpy(RefData.ref_data_raw, ref_data_raw, sizeof(RefData.ref_data_raw)); // Load referee data
        memcpy(packetBuffer + packetOffset, &RefData, sizeof(RefData));           // Copy the RefData into the buffer
        packetOffset += sizeof(RefData);

        // // Print Referee Data for debugging
        // Serial.println("Packing Referee Data:");
        // for (int i = 0; i < 180; i++)
        // {
        //     Serial.print(RefData.ref_data_raw[i]);
        //     Serial.print(" ");
        // }
        // Serial.println();

        // Copy CAN data into the buffer
        memcpy(packetBuffer + packetOffset, canData, sizeof(CANData));
        packetOffset += sizeof(CANData);

        // // Print CAN data before packing for debugging
        // Serial.println("CAN Data before packed:");
        // for (int i = 0; i < 2; i++)
        // {
        //     for (int j = 0; j < 8; j++)
        //     {
        //         for (int k = 0; k < 8; k++)
        //         {
        //             Serial.print(canData->data[i][j][k]);
        //             Serial.print(" ");
        //         }
        //         Serial.println();
        //     }
        //     Serial.println();
        // }

        // Serialize sensor data
        for (int i = 0; i < buff_sensor_count; i++) {

            estimatorManager.getBuffSensor(i).serialize(packetBuffer, packetOffset);
            //Serial.print("Buff Encoder ");
            // Serial.print(estimatorManager.getBuffSensor(i).getId());
            // Serial.print(": ");
            // estimatorManager.getBuffSensor(i).print();
            // Serial.println("Ending buff encoder serialization");
        }

        for (int i = 0; i < icm_sensor_count; i++) {
            //Serial.println("Starting ICM sensor serialization");
            estimatorManager.getICMSensor(i).serialize(packetBuffer, packetOffset);
            // Serial.print("ICM Sensor ");
            // Serial.print(estimatorManager.getICMSensor(i).getId());
            // Serial.print(": ");
            // estimatorManager.getICMSensor(i).print();
            // Serial.println("Ending ICM sensor serialization");
        }

        for (int i = 0; i < rev_sensor_count; i++) {
            //Serial.println("Starting rev encoder serialization");
            //Serial.print("Rev Encoder ");
            //Serial.print(estimatorManager.getRevSensor(i).getId());
            //Serial.print(": ");
            //estimatorManager.getRevSensor(i).print();
            estimatorManager.getRevSensor(i).serialize(packetBuffer, packetOffset);
            //Serial.println("Ending rev encoder serialization");
        }

        for (int i = 0; i < tof_sensor_count; i++) {
            // Serial.println("Starting TOF sensor serialization");
            // Serial.print("TOF Sensor ");
            // Serial.print(estimatorManager.getTOFSensor(i).getId());
            // Serial.print(": ");
            // estimatorManager.getTOFSensor(i).print();
            estimatorManager.getTOFSensor(i).serialize(packetBuffer, packetOffset);
            // Serial.println("Ending TOF sensor serialization");
        }

        // Serialize LiDAR sensors if they exist
        if (lidar_sensor_count == 2) {
            // Serial.print("Lidar 1: ");
            // lidar1.print_latest_packet();
            lidar1.serialize(packetBuffer, packetOffset);

            // Serial.print("Lidar 2: ");
            // lidar2.print_latest_packet();
            lidar2.serialize(packetBuffer, packetOffset);
        }
        //Serial.println("Data Packet Packed");
    }

    /// @brief Function to unpack data from a packet buffer
    /// @param packetBuffer Buffer containing the packed data
    void unpackDataPacket(uint8_t packetBuffer[BUFFER_SIZE]) {
        //Serial.println("Inside Unpacking Data Packet");
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

        // // Print the unpacked data for debugging
        // Serial.println("Unpacked Data:");
        // Serial.print("Timestamp: ");
        // Serial.println(timestamp);
        // Serial.print("Buff Sensor Count: ");
        // Serial.println(buff_sensor_count);
        // Serial.print("ICM Sensor Count: ");
        // Serial.println(icm_sensor_count);
        // Serial.print("Rev Sensor Count: ");
        // Serial.println(rev_sensor_count);
        // Serial.print("TOF Sensor Count: ");
        // Serial.println(tof_sensor_count);
        // Serial.print("Lidar Sensor Count: ");
        // Serial.println(lidar_sensor_count);

        // Unpack the RefereeData
        //Serial.println("Unpacking Referee Data:");
        memcpy(&refData, packetBuffer + packetOffset, sizeof(refData));
        packetOffset += sizeof(refData);

        // Print Referee Data for debugging
        //Serial.println("Referee Data:");
        for (int i = 0; i < 180; i++) {
            Serial.print(refData.ref_data_raw[i]);
            Serial.print(" ");
        }
        //Serial.println();

        // Unpack CAN data
        //Serial.println("Unpacking CAN Data:");
        memcpy(&canData, packetBuffer + packetOffset, sizeof(CANData));
        packetOffset += sizeof(CANData);

        // Print CAN data after unpacking for debugging
        //Serial.println("CAN Data unpacked:");
        // for (int i = 0; i < 2; i++)
        // {
        //     for (int j = 0; j < 8; j++)
        //     {
        //         for (int k = 0; k < 8; k++)
        //         {
        //             Serial.print(canData.data[i][j][k]);
        //             Serial.print(" ");
        //         }
        //         Serial.println();
        //     }
        //     Serial.println();
        // }

        // // Unpack the sensor data
        // //Serial.println("Unpacking Sensor Data:");

        // Unpack Buff Encoders
        for (int i = 0; i < buff_sensor_count; i++) {
            buff_sensors[i].id = packetBuffer[packetOffset++];
            memcpy(&buff_sensors[i].m_angle, packetBuffer + packetOffset, sizeof(buff_sensors[i].m_angle));
            packetOffset += sizeof(buff_sensors[i].m_angle);

            // Serial.print("Buff Encoder ");
            // Serial.print(buff_sensors[i].id);
            // Serial.print(": ");
            // buff_sensors[i].print();
        }

        // Unpack ICM Sensors
        for (int i = 0; i < icm_sensor_count; i++) {
            icm_sensors[i].deserialize(packetBuffer, packetOffset);
            // // Print the sensor data
            // Serial.print("ICM Sensor ");
            // Serial.print(icm_sensors[i].id);
            // Serial.print(": ");
            // icm_sensors[i].print();
        }

        // Unpack Rev Encoders
        for (int i = 0; i < rev_sensor_count; i++) {
            rev_sensors[i].deserialize(packetBuffer, packetOffset);
            // // Print the sensor data
            // Serial.print("Rev Encoder ");
            // Serial.print(rev_sensors[i].id);
            // Serial.print(": ");
            // rev_sensors[i].print();
        }

        // Unpack TOF Sensors
        for (int i = 0; i < tof_sensor_count; i++) {
            tof_sensors[i].deserialize(packetBuffer, packetOffset);
            // Print the sensor data
            // Serial.print("TOF Sensor ");
            // Serial.print(tof_sensors[i].id);
            // Serial.print(": ");
            // tof_sensors[i].print();
        }

        // Unpack LiDAR Sensors
        if (lidar_sensor_count == 2) {
            lidar_sensors[0].deserialize(packetBuffer, packetOffset);
            // Serial.print("Lidar 1: ");
            // lidar_sensors[0].print_latest_packet();

            lidar_sensors[1].deserialize(packetBuffer, packetOffset);
            // Serial.print("Lidar 2: ");
            // lidar_sensors[1].print_latest_packet();
        }
        //Serial.println("Exiting Unpacking Data Packet");
    }
};