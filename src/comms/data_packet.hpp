#include <cstdint>
#include <state.hpp>
#include <vector>
#include <rm_can.hpp>
#include "Sensor.hpp"
#include "constants.hpp"
#include "estimator_manager.hpp"
#include "config_layer.hpp"
#include "d200.hpp" 


struct RefereeData
{
    uint8_t ref_data_raw[180] = {0}; // construct ref data packet
};

struct buff_sensor
{
    uint8_t id;
    float m_angle;

    void print()
    {
        Serial.println("Buff Encoder:");
        Serial.print("Angle: ");
        Serial.println(m_angle);
    }
};

struct icm_sensor
{
    uint8_t id;
    float accel_X;
    float accel_Y;
    float accel_Z;
    float gyro_X;
    float gyro_Y;
    float gyro_Z;
    float temperature;

    void deserialize(const uint8_t *data, size_t &offset)
    {
        // code that prints out raw bytes of the buffer for this sensor
        Serial.print("ICM20649: ");
        for (int i = 0; i < 28; i++)
        {
            for (int ii = 0; ii <= 7; ii++)
            {
                int k = data[i] >> ii;
                if (k & 1)
                {
                    Serial.print("1");
                }
                else
                {
                    Serial.print("0");
                }
                Serial.print(" ");
            }
            Serial.println(" ");
        }
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

    void print()
    {
        // Display the temperature data, measured in Celcius
        Serial.print("\t\tTemperature ");
        Serial.print(temperature);
        Serial.println(" deg C");
        // Display the acceleration data, measured in m/s^2)
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

struct rev_sensor
{
    uint8_t id;
    int ticks;
    float radians;

    void deserialize(const uint8_t *data, size_t &offset)
    {
        id = data[offset++];
        memcpy(&ticks, data + offset, sizeof(ticks));
        offset += sizeof(ticks);
        memcpy(&radians, data + offset, sizeof(radians));
        offset += sizeof(radians);
    }

    void print()
    {
        Serial.println("Rev Encoder:");
        Serial.print("Ticks: ");
        Serial.println(ticks);
        Serial.print("Radians: ");
        Serial.println(radians);
    }
};

struct tof_sensor
{
    uint8_t id;
    uint16_t latest_distance;

    /// @brief function to deserialize the TOF sensor data
    /// @param data  data to deserialize
    /// @param offset  offset to deserialize the data
    void deserialize(const uint8_t *data, size_t &offset)
    {
        // deserialize the sensor id
        id = data[offset++];

        // deserialize the distance
        latest_distance = data[offset++];
        latest_distance |= data[offset++] << 8;
    }

    void print()
    {
        Serial.println("TOF Sensor:");
        Serial.printf("\tDistance: %u mm\n", latest_distance);
    }
};

struct lidar_sensor
{
    uint8_t id;
    int current_packet;
    D200Calibration cal;
    LidarDataPacketSI packets[D200_NUM_PACKETS_CACHED];
    void deserialize(const uint8_t *data, size_t &offset)
    {
        id = data[offset++];
        memcpy(&current_packet, data + offset, sizeof(current_packet));
        offset += sizeof(current_packet);
        memcpy(&cal, data + offset, sizeof(cal));
        offset += sizeof(cal);
        memcpy(packets, data + offset, D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE);
        offset += D200_NUM_PACKETS_CACHED * D200_PAYLOAD_SIZE;
    }

    void print_latest_packet()
    {
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

struct data_packet
{
    const Config *config; // Config struct to store all config data

    uint32_t timestamp; // Current time in microseconds
    State state;        // Robot state

    RefereeData refData;
    CANData canData;

    int buff_sensor_count;
    int icm_sensor_count;
    int rev_sensor_count;
    int tof_sensor_count;
    int lidar_sensor_count;

    buff_sensor *buff_sensors;
    icm_sensor *icm_sensors;
    rev_sensor *rev_sensors;
    tof_sensor *tof_sensors;

    lidar_sensor *lidar_sensors;

    // DataPacketHeader getHeader() const { return header; }
    RefereeData getRefData() const { return refData; }
    CANData getCanData() const { return canData; }

    data_packet(const Config *config_data)
    {

        config = config_data; // store the config data
        buff_sensor_count = config_data->num_sensors[0];
        icm_sensor_count = config_data->num_sensors[1];
        rev_sensor_count = config_data->num_sensors[2];
        tof_sensor_count = config_data->num_sensors[3];
        lidar_sensor_count = config_data->num_sensors[4];

        buff_sensors = new buff_sensor[buff_sensor_count];
        icm_sensors = new icm_sensor[icm_sensor_count];
        rev_sensors = new rev_sensor[rev_sensor_count];
        tof_sensors = new tof_sensor[tof_sensor_count];
        lidar_sensors = new lidar_sensor[lidar_sensor_count];

        // give the initial id of 254 to all sensors
        for (int i = 0; i < buff_sensor_count; ++i)
        {
            buff_sensors[i].id = 254;
        }
        for (int i = 0; i < icm_sensor_count; ++i)
        {
            icm_sensors[i].id = 254;
        }
        for (int i = 0; i < rev_sensor_count; ++i)
        {
            rev_sensors[i].id = 254;
        }
        for (int i = 0; i < tof_sensor_count; ++i)
        {
            tof_sensors[i].id = 254;
        }
        for (int i = 0; i < lidar_sensor_count; ++i)
        {
            lidar_sensors[i].id = 254;
        }
    }

    ~data_packet()
    {
        delete[] buff_sensors;
        delete[] icm_sensors;
        delete[] rev_sensors;
        delete[] tof_sensors;
    }

    void packDataPacket(uint8_t packetBuffer[BUFFER_SIZE], State robotState, uint8_t ref_data_raw[180], CANData *canData, EstimatorManager &estimatorManager, D200LD14P &lidar1, D200LD14P &lidar2)
    {
        size_t packetOffset = 0;

        // // Create the packet header
        // DataPacketHeader header;
        // Serial.println("Packing Robot State:");
        // header.state = robotState;

        // pack the timestamp, state, and number of sensors
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

        // print packet information
        Serial.println("Packing data packet:");
        Serial.print("Timestamp: ");
        Serial.println(timestamp);
        // Serial.print("State: ")
        Serial.print("Buff Sensor Count: ");
        Serial.println(buff_sensor_count);
        Serial.print("ICM Sensor Count: ");
        Serial.println(icm_sensor_count);
        Serial.print("Rev Sensor Count: ");
        Serial.println(rev_sensor_count);
        Serial.print("TOF Sensor Count: ");
        Serial.println(tof_sensor_count);
        Serial.println("Lidar Sensor Count: ");
        Serial.println(lidar_sensor_count);

        // Create the RefereeData
        RefereeData RefData;
        memcpy(RefData.ref_data_raw, ref_data_raw, sizeof(RefData.ref_data_raw)); // load ref_data_raw with the given parameter
        memcpy(packetBuffer + packetOffset, &RefData, sizeof(RefData));           // Copy the RefData into the buffer
        packetOffset += sizeof(RefData);

        Serial.println("Packing Referee Data:");
        for (int i = 0; i < 180; i++)
        {
            Serial.print(RefData.ref_data_raw[i]);
            Serial.print(" ");
        }
        Serial.println();

        // Copy CAN data into the buffer
        memcpy(packetBuffer + packetOffset, canData, sizeof(CANData));
        packetOffset += sizeof(CANData);

        // Print CAN data before packing
        Serial.println("CAN Data before packed:");
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                for (int k = 0; k < 8; k++)
                {
                    Serial.print(canData->data[i][j][k]);
                    Serial.print(" ");
                }
                Serial.println();
            }
            Serial.println();
        }

        // Serialize sensor data
        Serial.println("Sensor data before serialization:");
        for (int i = 0; i < buff_sensor_count; i++)
        {
            Serial.println("starting buff encoder serialization");
            estimatorManager.getBuffSensor(i).serialize(packetBuffer, packetOffset);
            Serial.print("Buff Encoder ");
            Serial.print(estimatorManager.getBuffSensor(i).getId());
            Serial.print(": ");
            estimatorManager.getBuffSensor(i).print();
            Serial.println("ending buff encoder serialization");
        }

        for (int i = 0; i < icm_sensor_count; i++)
        {
            Serial.println("starting icm sensor serialization");
            estimatorManager.getICMSensor(i).serialize(packetBuffer, packetOffset);
            Serial.print("ICM Sensor ");
            Serial.print(estimatorManager.getICMSensor(i).getId());
            Serial.print(": ");
            estimatorManager.getICMSensor(i).print();
            Serial.println("ending icm sensor serialization");
        }

        Serial.println("starting rev encoder serialization");
        Serial.print("Number of rev sensors: ");
        Serial.println(rev_sensor_count);

        for (int i = 0; i < rev_sensor_count; i++)
        {
            Serial.println("starting rev encoder serialization");
            Serial.print("Rev Encoder ");
            Serial.print(estimatorManager.getRevSensor(i).getId());
            Serial.print(": ");
            estimatorManager.getRevSensor(i).print();
            estimatorManager.getRevSensor(i).serialize(packetBuffer, packetOffset);
            Serial.println("ending rev encoder serialization");
        }

        for (int i = 0; i < tof_sensor_count; i++)
        {
            Serial.println("starting tof sensor serialization");
            Serial.print("TOF Sensor ");
            Serial.print(estimatorManager.getTOFSensor(i).getId());
            Serial.print(": ");
            estimatorManager.getTOFSensor(i).print();
            estimatorManager.getTOFSensor(i).serialize(packetBuffer, packetOffset);
            Serial.println("ending tof sensor serialization");
        }

        if (lidar_sensor_count == 2)
        {
            Serial.print("Lidar 1: ");
            lidar1.print_latest_packet();
            lidar1.serialize(packetBuffer, packetOffset);

            Serial.print("Lidar 2: ");
            lidar2.print_latest_packet();
            lidar2.serialize(packetBuffer, packetOffset);
        }
        Serial.println("Data Packet Packed");
    }

    void unpackDataPacket(uint8_t packetBuffer[BUFFER_SIZE])
    {
        Serial.println("Inside Unpacking Data Packet");
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

        // Unpack the RefereeData
        Serial.println("Unpacking Referee Data:");
        memcpy(&refData, packetBuffer + packetOffset, sizeof(refData));
        packetOffset += sizeof(refData);

        Serial.println("Referee Data:");
        for (int i = 0; i < 180; i++)
        {
            Serial.print(refData.ref_data_raw[i]);
            Serial.print(" ");
        }
        Serial.println();

        // unpack can data
        Serial.println("Unpacking CAN Data:");
        memcpy(&canData, packetBuffer + packetOffset, sizeof(CANData));
        packetOffset += sizeof(CANData);

        // Print CAN data after unpacking
        Serial.println("CAN Data unpacked:");
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                for (int k = 0; k < 8; k++)
                {
                    Serial.print(canData.data[i][j][k]);
                    Serial.print(" ");
                }
                Serial.println();
            }
            Serial.println();
        }

        // Unpack the sensor data
        Serial.println("Unpacking Sensor Data:");
        // Unpack Buff Encoders
        for (int i = 0; i < buff_sensor_count; i++)
        {
            buff_sensors[i].id = packetBuffer[packetOffset++];
            memcpy(&buff_sensors[i].m_angle, packetBuffer + packetOffset, sizeof(buff_sensors[i].m_angle));
            packetOffset += sizeof(buff_sensors[i].m_angle);

            Serial.print("Buff Encoder ");
            Serial.print(buff_sensors[i].id);
            Serial.print(": ");
            buff_sensors[i].print();
        }

        // Unpack ICM Sensors
        for (int i = 0; i < icm_sensor_count; i++)
        {

            icm_sensors[i].deserialize(packetBuffer, packetOffset);
            // print the sensor data
            Serial.print("ICM Sensor ");
            Serial.print(icm_sensors[i].id);
            Serial.print(": ");
            icm_sensors[i].print();
        }

        // Unpack Rev Encoders
        for (int i = 0; i < rev_sensor_count; i++)
        {

            rev_sensors[i].deserialize(packetBuffer, packetOffset);
            // print the sensor data
            Serial.print("Rev Encoder ");
            Serial.print(rev_sensors[i].id);
            Serial.print(": ");
            rev_sensors[i].print();
        }

        // Unpack TOF Sensors
        for (int i = 0; i < tof_sensor_count; i++)
        {

            tof_sensors[i].deserialize(packetBuffer, packetOffset);
            // print the sensor data
            Serial.print("TOF Sensor ");
            Serial.print(tof_sensors[i].id);
            Serial.print(": ");
            tof_sensors[i].print();
        }

        // Unpack Lidar Sensors
        if (lidar_sensor_count == 2)
        {
            lidar_sensors[0].deserialize(packetBuffer, packetOffset);
            Serial.print("Lidar 1: ");
            lidar_sensors[0].print_latest_packet();

            lidar_sensors[1].deserialize(packetBuffer, packetOffset);
            Serial.print("Lidar 2: ");
            lidar_sensors[1].print_latest_packet();
        }
        Serial.println("Exiting Unpacking Data Packet");
    }
};
