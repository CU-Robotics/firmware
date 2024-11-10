#ifndef VIRTUAL_SENSOR_MANAGER_HPP
#define VIRTUAL_SENSOR_MANAGER_HPP


#include "buff_encoder.hpp"
#include "ICM20649.hpp"
#include "rev_encoder.hpp"
#include "TOFSensor.hpp"
#include "d200.hpp"
#include "config_layer.hpp"
#include "constants.hpp"

class VirtualSensorManager
{
private:
    BuffEncoder* buff_sensors;
    ICM20649* icm_sensors;
    RevEncoder* rev_sensors;
    TOFSensor* tof_sensors;
    D200LD14P lidar1;
    D200LD14P lidar2;
    int buff_sensor_count;
    int icm_sensor_count;
    int rev_sensor_count;
    int tof_sensor_count;

public:
    VirtualSensorManager(const Config* config_data)
    {
        buff_sensor_count = config_data->num_sensors[0];
        icm_sensor_count = config_data->num_sensors[1];
        rev_sensor_count = config_data->num_sensors[2];
        tof_sensor_count = config_data->num_sensors[3];

        buff_sensors = new BuffEncoder[buff_sensor_count];
        icm_sensors = new ICM20649[icm_sensor_count];
        rev_sensors = new RevEncoder[rev_sensor_count];
        tof_sensors = new TOFSensor[tof_sensor_count];


        //give the initial id of -1 to all sensors
        for (int i = 0; i < buff_sensor_count; ++i)
        {
            buff_sensors[i].setId(254);
        }
        for (int i = 0; i < icm_sensor_count; ++i)
        {
            icm_sensors[i].setId(254);
        }
        for (int i = 0; i < rev_sensor_count; ++i)
        {
            rev_sensors[i].setId(254);
        }
        for (int i = 0; i < tof_sensor_count; ++i)
        {
            tof_sensors[i].setId(254);
        }
    }

    ~VirtualSensorManager()
    {
        delete[] buff_sensors;
        delete[] icm_sensors;
        delete[] rev_sensors;
        delete[] tof_sensors;
    }

    BuffEncoder* getBuffSensors() { return buff_sensors; }
    ICM20649* getICMSensors() { return icm_sensors; }
    RevEncoder* getRevSensors() { return rev_sensors; }
    TOFSensor* getTOFSensors() { return tof_sensors; }
    D200LD14P& getLidar1() { return lidar1; }
    D200LD14P& getLidar2() { return lidar2; }

    int getBuffSensorCount() { return buff_sensor_count; }
    int getICMSensorCount() { return icm_sensor_count; }
    int getRevSensorCount() { return rev_sensor_count; }
    int getTOFSensorCount() { return tof_sensor_count; }


    void updateSensor(SensorType type, int id, void* sensor)
    {
        switch (type)
        {
        case SensorType::BUFFENC:
            for (int i = 0; i < buff_sensor_count; ++i)
            {
                if (buff_sensors[i].getId() == 254 || buff_sensors[i].getId() == id)
                {
                    buff_sensors[i] = *static_cast<BuffEncoder*>(sensor);
                    buff_sensors[i].setId(id);
                    break;
                }
            }
            break;
        case SensorType::ICM:
            for (int i = 0; i < icm_sensor_count; ++i)
            {
                if (icm_sensors[i].getId() == 254 || icm_sensors[i].getId() == id)
                {
                    icm_sensors[i] = *static_cast<ICM20649*>(sensor);
                    icm_sensors[i].setId(id);
                    break;
                }
            }
            break;
        case SensorType::REVENC:
            for (int i = 0; i < rev_sensor_count; ++i)
            {
                if (rev_sensors[i].getId() == 254 || rev_sensors[i].getId() == id)
                {
                    rev_sensors[i] = *static_cast<RevEncoder*>(sensor);
                    rev_sensors[i].setId(id);
                    break;
                }
            }
            break;
        case SensorType::TOF:
            for (int i = 0; i < tof_sensor_count; ++i)
            {
                if (tof_sensors[i].getId() == 254 || tof_sensors[i].getId() == id)
                {
                    tof_sensors[i] = *static_cast<TOFSensor*>(sensor);
                    tof_sensors[i].setId(id);
                    break;
                }
            }
            break;
        case SensorType::LIDAR:
            if (id == 1)
            {
                lidar1 = *static_cast<D200LD14P*>(sensor);
            }
            else if (id == 2)
            {
                lidar2 = *static_cast<D200LD14P*>(sensor);
            }
            break;
        default:
            break;
        }
    }
};

#endif // VIRTUAL_SENSOR_MANAGER_HPP