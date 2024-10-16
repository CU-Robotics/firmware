#include <cstdint>
#include <state.hpp>
#include <vector>

#define MAX_PACKET_SIZE 4096

enum sensorType
    {
        LIDAR,
        TOF,
        OTHERS
    };



struct data_packet
{

    char packetBuffer[MAX_PACKET_SIZE];

    std::vector<uint8_t> sensorDataBuffer;
    

#pragma pack(push, 1)
    typedef struct
    {
        uint32_t timestamp;  // Current time in microseconds
        State state;         // Robot state
        uint16_t numSensors ; // Number of sensors
        uint16_t sensorDataBufferLength;
    } DataPacketHeader;


    //sensor data will follow Sensor Data Header
    typedef struct
    {
        uint16_t sensorType; // Identifier for sensor type
        uint16_t dataLength; // Length of the sensor data in bytes
    } SensorDataHeader;



    typedef struct
    {
        uint8_t ref_data_raw[180] = {0}; // construct ref data packet
    } RefereeData;

    typedef struct
    {

    } CANData;

    typedef struct
    {

    } LoggingData;
#pragma pack(pop)

    std::vector<SensorDataHeader> all_sensor_headers;











    // void addSensorData(sensorType sensor_type, uint16_t dataLength,std::vector<uint8_t> sensorData ) {
    //     SensorDataHeader new_sensor = {sensor_type,dataLength};
    //     all_sensor_headers.push_back(new_sensor);
    //     sensorData.insert(std::end(sensorDataBuffer),std::begin(sensorData),std::end(sensorData)); //append sensorData onto sensorDataBuffer
    // }

    void packDataPacket(uint8_t *packetBuffer, uint8_t numOfSensors, State robotState, uint8_t ref_data_raw[180])
    {
        int packetOffset = 0;
        // Create the packet header
        DataPacketHeader header;
        header.state = robotState;
        header.timestamp = micros();
        header.numSensors = all_sensor_headers.size();
        header.sensorDataBufferLength = sensorDataBuffer.size();

        RefereeData RefData;                                              // Create the ReferreData
        memcpy(RefData.ref_data_raw, ref_data_raw, sizeof(ref_data_raw)); // load ref_data_raw with the given parameter
        packetOffset += sizeof(ref_data_raw);


        // Copy the packet header into the buffer
        memcpy(packetBuffer + packetOffset, &header, sizeof(header));
        packetOffset += sizeof(header);
        

        int sensorDataBufferOffset = 0;
        
        //for every sensor
        for(int i = 0; i < header.numSensors; i++) {
            //add the sensorDataHeader
            memcpy(packetBuffer + packetOffset, &all_sensor_headers.at(i), sizeof(all_sensor_headers.at(i)));
            packetOffset += sizeof(all_sensor_headers.at(i));
            //then pack the sensors associated data
            memcpy(packetBuffer + packetOffset, &sensorDataBuffer.at(sensorDataBufferOffset), all_sensor_headers.at(i).dataLength);
            sensorDataBufferOffset += all_sensor_headers.at(i).dataLength;
        }

        
    }
};
