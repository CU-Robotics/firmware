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
#include "virtual_sensor_manager.hpp"

struct DataPacketHeader
    {
        uint32_t timestamp;  // Current time in microseconds
        State state;         // Robot state
        uint16_t numSensors; // Number of sensors
        uint16_t sensorDataBufferLength;
    } ;

    struct RefereeData
    {
        uint8_t ref_data_raw[180] = {0}; // construct ref data packet
    } ;


struct data_packet
{


    DataPacketHeader header;
    RefereeData refData;
    CANData canData;

    D200LD14P lidar1;
    D200LD14P lidar2;

    DataPacketHeader getHeader() const { return header; }
    RefereeData getRefData() const { return refData; }
    CANData getCanData() const { return canData; }

    void packDataPacket(uint8_t packetBuffer[BUFFER_SIZE], State robotState, uint8_t ref_data_raw[180], CANData *canData, const Config* config_data, EstimatorManager& estimatorManager, D200LD14P& lidar1, D200LD14P& lidar2) 
    {
        size_t packetOffset = 0;
    
        // Create the packet header
        DataPacketHeader header;
        Serial.println("Packing Robot State:");
        header.state = robotState;
        
        header.timestamp = micros();
        header.numSensors = config_data->num_sensors[0] + config_data->num_sensors[1] + config_data->num_sensors[2] + config_data->num_sensors[3] + config_data->num_sensors[4];
    

        // Print header information
        Serial.println("Packing Header:");
        Serial.println("Timestamp: ");
        Serial.println(header.timestamp);
        Serial.println("Num Sensors: ");
        Serial.println(header.numSensors);
    
        // Copy the packet header into the buffer
        memcpy(packetBuffer + packetOffset, &header, sizeof(header));
        packetOffset += sizeof(header);
    
        

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
            Serial.println("starting buff encoder serialization");
            BuffEncoder encoder = estimatorManager.getBuffSensor(i);
            Serial.print("Buff Encoder ");
            Serial.print(encoder.getId());
            Serial.print(": ");
            encoder.print();
            encoder.serialize(packetBuffer, packetOffset);
            Serial.println("ending buff encoder serialization");
        }

        for(int i = 0; i < config_data->num_sensors[1]; i++)
        {
            Serial.println("starting icm sensor serialization");
            ICM20649 icm = estimatorManager.getICMSensor(i);
            Serial.print("ICM Sensor ");
            Serial.print(icm.getId());
            Serial.print(": ");
            icm.print();
            icm.serialize(packetBuffer, packetOffset);
            Serial.println("ending icm sensor serialization");
        }

        Serial.println("starting rev encoder serialization");
        Serial.print("Number of rev sensors: ");
        Serial.println(config_data->num_sensors[2]);

        for(int i = 0; i < config_data->num_sensors[2]; i++)
        {
            Serial.println("starting rev encoder serialization");
            RevEncoder rev = estimatorManager.getRevSensor(i);
            Serial.print("Rev Encoder ");
            Serial.print(rev.getId());
            Serial.print(": ");
            rev.print();
            rev.serialize(packetBuffer, packetOffset);
            Serial.println("ending rev encoder serialization");
        }

        for(int i = 0; i < config_data->num_sensors[3]; i++) {
            Serial.println("starting tof sensor serialization");
            TOFSensor tof = estimatorManager.getTOFSensor(i);
            Serial.print("TOF Sensor ");
            Serial.print(tof.getId());
            Serial.print(": ");
            tof.print();
            tof.serialize(packetBuffer, packetOffset);
            Serial.println("ending tof sensor serialization");
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
        Serial.println("Data Packet Packed");
    }

    void unpackDataPacket(uint8_t packetBuffer[BUFFER_SIZE], const Config* config_data, VirtualSensorManager& virtualSensorManager)
    {
        Serial.println("Inside Unpacking Data Packet");
        size_t packetOffset = 0;

        // Unpack the header
        Serial.println("Unpacking Header:");
        memcpy(&header, packetBuffer + packetOffset, sizeof(header));
        packetOffset += sizeof(header);

        // Print header information
        Serial.println("Timestamp: ");
        Serial.println(header.timestamp);
        Serial.println("Num Sensors: ");
        Serial.println(header.numSensors);

        // Unpack the RefereeData
        Serial.println("Unpacking Referee Data:");
        memcpy(&refData, packetBuffer + packetOffset, sizeof(refData));
        packetOffset += sizeof(refData);

        Serial.println("Referee Data:");
        for (int i = 0; i < 180; i++) {
            Serial.print(refData.ref_data_raw[i]);
            Serial.print(" ");
        }
        Serial.println();

        //unpack can data
        Serial.println("Unpacking CAN Data:");
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
        Serial.println("Unpacking Sensor Data:");
        // Unpack Buff Encoders
        for(int i = 0; i < config_data->num_sensors[0]; i++)
        {
            BuffEncoder encoder;
            encoder.deserialize(packetBuffer, packetOffset);
            
            virtualSensorManager.updateSensor(SensorType::BUFFENC, encoder.getId(), &encoder);

            Serial.print("Buff Encoder ");
            Serial.print(virtualSensorManager.getBuffSensors()[i].getId());
            Serial.print(": ");
            //buff_sensors[i].print(); 

            virtualSensorManager.getBuffSensors()[i].print();
        }

        // Unpack ICM Sensors
        for(int i = 0; i < config_data->num_sensors[1]; i++)
        {
            // icm_sensors[i].deserialize(packetBuffer, packetOffset);
            // Serial.print("ICM Sensor ");
            // Serial.print(icm_sensors[i].getId());  
            // Serial.print(": ");
            // icm_sensors[i].print(); 

            ICM20649 icm;
            icm.deserialize(packetBuffer, packetOffset);
            virtualSensorManager.updateSensor(SensorType::ICM, icm.getId(), &icm);

            Serial.print("ICM Sensor ");
            Serial.print(virtualSensorManager.getICMSensors()[i].getId());
            Serial.print(": ");
            virtualSensorManager.getICMSensors()[i].print();
        }

        // Unpack Rev Encoders
        for(int i = 0; i < config_data->num_sensors[2]; i++)
        {
            // rev_sensors[i].deserialize(packetBuffer, packetOffset);
            // Serial.print("Rev Encoder ");
            // Serial.print(rev_sensors[i].getId());
            // Serial.print(": ");
            // rev_sensors[i].print(); 

            RevEncoder rev;
            rev.deserialize(packetBuffer, packetOffset);
            virtualSensorManager.updateSensor(SensorType::REVENC, rev.getId(), &rev);

            Serial.print("Rev Encoder ");
            Serial.print(virtualSensorManager.getRevSensors()[i].getId());
            Serial.print(": ");
            virtualSensorManager.getRevSensors()[i].print();
        }

        // Unpack TOF Sensors
        for(int i = 0; i < config_data->num_sensors[3]; i++)
        {
            // tof_sensors[i].deserialize(packetBuffer, packetOffset);
            // Serial.print("TOF Sensor ");
            // Serial.print(tof_sensors[i].getId());
            // Serial.print(": ");
            // tof_sensors[i].print();

            TOFSensor tof;
            tof.deserialize(packetBuffer, packetOffset);
            virtualSensorManager.updateSensor(SensorType::TOF, tof.getId(), &tof);

            Serial.print("TOF Sensor ");
            Serial.print(virtualSensorManager.getTOFSensors()[i].getId());
            Serial.print(": ");
            virtualSensorManager.getTOFSensors()[i].print();

        }

        // Unpack Lidar Sensors
        if(config_data->num_sensors[4] == 2)
        {
            // lidar1.deserialize(packetBuffer, packetOffset);
            // Serial.print("Lidar 1: ");
            // lidar1.print_latest_packet();

            // lidar2.deserialize(packetBuffer, packetOffset);
            // Serial.print("Lidar 2: ");
            // lidar2.print_latest_packet();


            virtualSensorManager.updateSensor(SensorType::LIDAR, 1, &lidar1);
            virtualSensorManager.updateSensor(SensorType::LIDAR, 2, &lidar2);

            Serial.print("Lidar 1: ");
            virtualSensorManager.getLidar1().print_latest_packet();

            Serial.print("Lidar 2: ");
            virtualSensorManager.getLidar2().print_latest_packet();
        }
        Serial.println("Exiting Unpacking Data Packet");
    }

};

