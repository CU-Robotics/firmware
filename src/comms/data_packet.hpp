#include <cstdint>
#include <state.hpp>
#include <vector>
#include <rm_can.hpp>
#include "Sensor.hpp"
#include "constants.hpp"
#include "estimator_manager.hpp"
#include "config_layer.hpp"
#include "buff_encoder.hpp"
#include "ICM20649.hpp"
#include "rev_encoder.hpp"
#include "d200.hpp"

struct DataPacketHeader
    {
        uint32_t timestamp;  // Current time in microseconds
        State state;         // Robot state
        uint16_t numSensors; // Number of sensors
        uint16_t sensorDataBufferLength;
    } ;

struct data_packet
{


    struct RefereeData
    {
        uint8_t ref_data_raw[180] = {0}; // construct ref data packet
    } ;

    struct LoggingData
    {

    } ;

    //these are temporary, I need to figure out where to send the unpacked sensor data
    BuffEncoder buff_sensors[MAX_SENSORS];
    ICM20649 icm_sensors[MAX_SENSORS];
    RevEncoder rev_sensors[MAX_SENSORS];
    TOFSensor tof_sensors[MAX_SENSORS];
    D200LD14P lidar1;
    D200LD14P lidar2;

    void packDataPacket(uint8_t packetBuffer[BUFFER_SIZE], State robotState, uint8_t ref_data_raw[180], CANData *canData, const Config* config_data, EstimatorManager& estimatorManager, D200LD14P& lidar1, D200LD14P& lidar2) 
    {
        size_t packetOffset = 0;
    
        // Create the packet header
        DataPacketHeader header;
        header.state = robotState;
        header.timestamp = micros();
        header.numSensors = config_data->num_sensors[0] + config_data->num_sensors[1] + config_data->num_sensors[2] + config_data->num_sensors[3] + config_data->num_sensors[4];
    
        // Copy the packet header into the buffer
        memcpy(packetBuffer + packetOffset, &header, sizeof(header));
        packetOffset += sizeof(header);
    
        // Print header information
        Serial.println("Packing Header:");
        Serial.println("Timestamp: ");
        Serial.println(header.timestamp);
        Serial.println("Num Sensors: ");
        Serial.println(header.numSensors);
    
        // Create the RefereeData
        RefereeData RefData;
        memcpy(RefData.ref_data_raw, ref_data_raw, sizeof(RefData.ref_data_raw)); // load ref_data_raw with the given parameter
        memcpy(packetBuffer + packetOffset, &RefData, sizeof(RefData)); // Copy the RefData into the buffer
        packetOffset += sizeof(RefData);
    
        Serial.println("Packing Referee Data:");
        for (int i = 0; i < 180; i++) {
            Serial.print(RefData.ref_data_raw[i]);
            Serial.print(" ");
        }
        Serial.println();
    
        // Copy CAN data into the buffer
        memcpy(packetBuffer + packetOffset, canData, sizeof(CANData));
        packetOffset += sizeof(CANData);
    
        // Print CAN data before packing
        Serial.println("CAN Data before packed:");
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 8; j++) {
                for (int k = 0; k < 8; k++) {
                    Serial.print(canData->data[i][j][k]);
                    Serial.print(" ");
                }
                Serial.println();
            }
            Serial.println();
        }

        // Serialize sensor data
        Serial.println("Sensor data before serialization:");
        for(int i = 0; i < config_data->num_sensors[0]; i++)
        {
            BuffEncoder encoder = estimatorManager.getBuffSensor(i);
            Serial.print("Buff Encoder ");
            Serial.print(encoder.getId());
            Serial.print(": ");
            encoder.print();
            encoder.serialize(packetBuffer, packetOffset);
        }

        for(int i = 0; i < config_data->num_sensors[1]; i++)
        {
            ICM20649 icm = estimatorManager.getICMSensor(i);
            Serial.print("ICM Sensor ");
            Serial.print(icm.getId());
            Serial.print(": ");
            icm.print();
            icm.serialize(packetBuffer, packetOffset);
        }

        for(int i = 0; i < config_data->num_sensors[2]; i++)
        {
            RevEncoder rev = estimatorManager.getRevSensor(i);
            Serial.print("Rev Encoder ");
            Serial.print(rev.getId());
            Serial.print(": ");
            rev.print();
            rev.serialize(packetBuffer, packetOffset);
        }

        for(int i = 0; i < config_data->num_sensors[3]; i++) {
            TOFSensor tof = estimatorManager.getTOFSensor(i);
            Serial.print("TOF Sensor ");
            Serial.print(tof.getId());
            Serial.print(": ");
            tof.print();
            tof.serialize(packetBuffer, packetOffset);
        }

        if(config_data->num_sensors[4] == 2)
        {
            Serial.print("Lidar 1: ");
            lidar1.print_latest_packet();
            lidar1.serialize(packetBuffer, packetOffset);

            Serial.print("Lidar 2: ");
            lidar2.print_latest_packet();
            lidar2.serialize(packetBuffer, packetOffset);
        }
    }

    void unpackDataPacket(uint8_t packetBuffer[BUFFER_SIZE], const Config* config_data)
    {
        size_t packetOffset = 0;

        // Unpack the header
        DataPacketHeader header;
        memcpy(&header, packetBuffer + packetOffset, sizeof(header));
        packetOffset += sizeof(header);

        // Print header information
        Serial.println("Timestamp: ");
        Serial.println(header.timestamp);
        Serial.println("Num Sensors: ");
        Serial.println(header.numSensors);

        // Unpack the RefereeData
        RefereeData refData;
        memcpy(&refData, packetBuffer + packetOffset, sizeof(refData));
        packetOffset += sizeof(refData);

        Serial.println("Referee Data:");
        for (int i = 0; i < 180; i++) {
            Serial.print(refData.ref_data_raw[i]);
            Serial.print(" ");
        }
        Serial.println();

        // Unpack CAN data
        CANData canData;
        memcpy(&canData, packetBuffer + packetOffset, sizeof(CANData));
        packetOffset += sizeof(CANData);

        // Print CAN data after unpacking
        Serial.println("CAN Data unpacked:");
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 8; j++) {
                for (int k = 0; k < 8; k++) {
                    Serial.print(canData.data[i][j][k]);
                    Serial.print(" ");
                }
                Serial.println();
            }
            Serial.println();
        }

        // Unpack the sensor data
        // Unpack Buff Encoders
        for(int i = 0; i < config_data->num_sensors[0]; i++)
        {
            buff_sensors[i].deserialize(packetBuffer, packetOffset);
            Serial.print("Buff Encoder ");
            Serial.print(buff_sensors[i].getId());
            Serial.print(": ");
            buff_sensors[i].print(); 
        }

        // Unpack ICM Sensors
        for(int i = 0; i < config_data->num_sensors[1]; i++)
        {
            icm_sensors[i].deserialize(packetBuffer, packetOffset);
            Serial.print("ICM Sensor ");
            Serial.print(icm_sensors[i].getId());  
            Serial.print(": ");
            icm_sensors[i].print(); 
        }

        // Unpack Rev Encoders
        for(int i = 0; i < config_data->num_sensors[2]; i++)
        {
            rev_sensors[i].deserialize(packetBuffer, packetOffset);
            Serial.print("Rev Encoder ");
            Serial.print(rev_sensors[i].getId());
            Serial.print(": ");
            rev_sensors[i].print(); 
        }

        // Unpack TOF Sensors
        for(int i = 0; i < config_data->num_sensors[3]; i++)
        {
            tof_sensors[i].deserialize(packetBuffer, packetOffset);
            Serial.print("TOF Sensor ");
            Serial.print(tof_sensors[i].getId());
            Serial.print(": ");
            tof_sensors[i].print();
        }

        // Unpack Lidar Sensors
        if(config_data->num_sensors[4] == 2)
        {
            lidar1.deserialize(packetBuffer, packetOffset);
            Serial.print("Lidar 1: ");
            lidar1.print_latest_packet();

            lidar2.deserialize(packetBuffer, packetOffset);
            Serial.print("Lidar 2: ");
            lidar2.print_latest_packet();
        }
    }
};

