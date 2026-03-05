#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include <concepts>
#include <map>
#include <memory>
#include <string>

#include "sensors/sensor.hpp"
#include "comms/config_data/robot_config.hpp"

/// @class SensorManager
/// @brief Class to manage sensors on the robot
class SensorManager {
public:

    /// @brief Constructor for the SensorManager class
    SensorManager();

    /// @brief Destructor for the SensorManager class
    ~SensorManager();

    /// @brief Initialize the sensor manager with configuration data
    void init(const Cfg::RobotConfig& config_data);

    void configure_sensors(const Cfg::RobotConfig& config_data);

    void initialize_sensors();

    /// @brief Read all sensor data and send to comms
    void read();

    void send_to_comms();
    
    template<typename SensorType>
    requires std::derived_from<SensorType, Sensor>
    std::shared_ptr<SensorType> SensorManager::get_sensor_by_name(Cfg::SensorName name) {
        auto it = sensors.find(name);
        if (it != sensors.end()) {
            std::shared_ptr<SensorType> sensor_ptr = std::dynamic_pointer_cast<SensorType>(it->second);
            if (sensor_ptr) {
                return sensor_ptr;
            } else {
                safety_procedure("SensorManager: Sensor with name " + std::to_string(static_cast<uint32_t>(name)) + " found but is not of the requested type");
            }
        } else {
            safety_procedure("SensorManager: Get sensor by name failed, no sensor with the given name found");
        }
    }
private:

    std::map<Cfg::SensorName, std::shared_ptr<Sensor>> sensors;
};