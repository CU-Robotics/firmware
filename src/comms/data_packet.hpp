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

struct DataPacketHeader
    {
        uint32_t timestamp;  // Current time in microseconds
        State state;         // Robot state
        uint16_t numSensors; // Number of sensors
        uint16_t sensorDataBufferLength;
    } ;

struct data_packet
{

    char packetBuffer[BUFFER_SIZE];

    std::vector<uint8_t> sensorDataBuffer;
    
    

    

    // sensor data will follow Sensor Data Header
    struct SensorDataHeader
    {
        SensorType sensorType; // Identifier for sensor types
        uint8_t id;            // Unique identifier for sensor
    } ;


    struct RefereeData
    {
        uint8_t ref_data_raw[180] = {0}; // construct ref data packet
    } ;

    struct LoggingData
    {

    } ;

    std::vector<SensorDataHeader> all_sensor_headers;
    //std::vector<Sensor> sensors;

    //these are temporary, I need to figure out where to send the unpacked sensor data
    BuffEncoder buff_sensors[MAX_SENSORS];
    ICM20649 icm_sensors[MAX_SENSORS];
    RevEncoder rev_sensors[MAX_SENSORS];

    void packDataPacket(uint8_t packetBuffer[BUFFER_SIZE], uint32_t timestamp, State robotState, uint8_t ref_data_raw[180], CANData *canData, const Config* config_data, EstimatorManager& estimatorManager) 
        
    
    {
        size_t packetOffset = 0;
    
        // Create the packet header
        DataPacketHeader header;
        header.state = robotState;
        header.timestamp = timestamp;
        //header.numSensors =;
    
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



        //go through num_sensors array from config and for each sensor, serialize it and add it to the packetBuffer
        
        //add every buff encider to the packetBuffer
        for(int i = 0; i < config_data->num_sensors[0]; i++)
        {
            BuffEncoder encoder = estimatorManager.getBuffSensor(i);
            encoder.serialize(packetBuffer, packetOffset);
        }

        //add every icm sensor to the packetBuffer
        for(int i = 0; i < config_data->num_sensors[1]; i++)
        {
            ICM20649 icm = estimatorManager.getICMSensor(i);
            icm.serialize(packetBuffer, packetOffset);
        }

        //add every rev encoder to the packetBuffer
        for(int i = 0; i < config_data->num_sensors[2]; i++)
        {
            RevEncoder rev = estimatorManager.getRevSensor(i);
            rev.serialize(packetBuffer, packetOffset);
        }
    }

        void unpackDataPacket(uint8_t packetBuffer[BUFFER_SIZE],const Config* config_data)
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
        //unpack buff encoders
        for(int i = 0; i < config_data->num_sensors[0]; i++)
        {
            buff_sensors[i].deserialize(packetBuffer, packetOffset);
        }

        //unpack icm sensors
        for(int i = 0; i < config_data->num_sensors[1]; i++)
        {
            icm_sensors[i].deserialize(packetBuffer, packetOffset);
        }

        //unpack rev encoders
        for(int i = 0; i < config_data->num_sensors[2]; i++)
        {
            rev_sensors[i].deserialize(packetBuffer, packetOffset);
        }
    }   

    
};

