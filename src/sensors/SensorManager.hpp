#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "Sensor.hpp"
#include "sensor_constants.hpp"
#include "config_layer.hpp"
//#include "estimator_manager.hpp"
#include "config_layer.hpp"
#include "d200.hpp"
#include "dr16.hpp"
#include "ICM20649.hpp"
#include <SPI.h>
#include "buff_encoder.hpp"
#include "TOFSensor.hpp"
#include "rev_encoder.hpp"
#include "sensors/d200.hpp"
#include <Arduino.h>


#define NUM_SENSOR_TYPE 16
#define NUM_IMU_CALIBRATION 50000




class SensorManager {
    public:
        Sensor* sensors[MAX_SENSORS];

        SensorManager();
        ~SensorManager();

        
        void init(const Config* config_data);

        void read();

        /// @brief get the specified buff encoder sensor from the array
        /// @param index index of the sensor object to get
        BuffEncoder* get_buff_encoder(int index);

        /// @brief get the specified icm sensor from the array
        /// @param index index of the sensor object to get
        ICM20649* get_icm_sensor(int index);

        /// @brief get the specified rev sensor from the array
        /// @param index index of the sensor object to get
        RevEncoder* get_rev_sensor(int index);

        /// @brief get the specified tof sensor from the array
        /// @param index index of the sensor object to get
        TOFSensor* get_tof_sensor(int index);

        // /// @brief get the specified lidar sensor from the array
        // /// @param index index of the sensor object to get
        // LidarSensor& get_lidar_sensor(int index);


        int get_num_sensors(SensorType sensor_type) {
            switch (sensor_type) {
                case SensorType::BUFFENC:
                    return buff_sensor_count;
                case SensorType::ICM:
                    return icm_sensor_count;
                case SensorType::REVENC:
                    return rev_sensor_count;
                case SensorType::TOF:
                    return tof_sensor_count;
                case SensorType::LIDAR:
                    return lidar_sensor_count;
                default:
                    return 0;
            }
        }

        void calibrate_imus();

        //TODO: Create function that will create a struct to work with the new comms

    private:
        /// Number of buff sensors.Buff sensor
    int buff_sensor_count;
    /// Number of ICM sensors.
    int icm_sensor_count;
    /// Number of Rev sensors.
    int rev_sensor_count;
    /// Number of TOF sensors.
    int tof_sensor_count;
    /// Number of LiDAR sensors.
    int lidar_sensor_count;

    /// @brief array to store the number of sensors for each sensor type
    int num_sensors[NUM_SENSORS];

    /// @brief array to store robot icm imu's
    ICM20649 icm_sensors[NUM_SENSOR_TYPE];
    /// @brief array to store robot buff encoders
    BuffEncoder buff_encoders[NUM_SENSOR_TYPE];
    /// @brief array to store robot rev encoders
    RevEncoder rev_sensors[NUM_SENSOR_TYPE];
    /// @brief array to store tof sensors
    TOFSensor tof_sensors[NUM_SENSOR_TYPE];

    D200LD14P lidar1 = D200LD14P(&Serial4, 0);
    D200LD14P lidar2 = D200LD14P(&Serial5, 1);

    
    /// Array of LiDAR sensor data structs.
    LidarSensorData* lidar_sensors_data;

};


#endif // SENSOR_MANAGER_HPP