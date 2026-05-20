#pragma once

#include <concepts>
#include <map>
#include <memory>
#include <string>

#include "sensors/sensor.hpp"
#include "comms/config_data/robot_config.hpp"

#include "utils/safety.hpp"

/// @class SensorManager
/// @brief Class to manage sensors on the robot
class SensorManager {
public:

    /// @brief Constructor for the SensorManager class
    SensorManager();

    /// @brief Destructor for the SensorManager class
    ~SensorManager();

    /// @brief Initialize the sensor manager with configuration data. This will configure and initialize all sensors.
    /// @param config_data The configuration data to use to initialize the sensor manager and all sensors
    void init(const Cfg::RobotConfig& config_data);

    /// @brief Configure the sensors based on the configuration data
    /// @param config_data The configuration data to use to configure the sensors
    void configure_sensors(const Cfg::RobotConfig& config_data);
    /// @brief Call each sensor's init function
    void initialize_sensors();
	/// @brief call each sensor's request_read function
	void request_read();
    /// @brief Call each sensor's read function to update their data
    void read();
    /// @brief Call each sensor's send_to_comms function to send their data to comms
    void send_to_comms();
    /// @brief Triggers the live dashboard for any supported sensors
    void print_sensors_live();
    /// @brief Get a sensor by its name and type. Will trigger safety procedure if the sensor is not found or is not of the requested type.
    /// @param name The name of the sensor to get
    /// @tparam SensorType The type of the sensor to get, must be derived from the Sensor class
    /// @return A shared pointer to the sensor with the requested name and type
    template<typename SensorType>
    requires std::derived_from<SensorType, Sensor>
    std::shared_ptr<SensorType> get_sensor_by_name(const Cfg::SensorName& name) {
        auto it = sensors.find(name);
        if (it != sensors.end()) {
            std::shared_ptr<SensorType> sensor_ptr = std::dynamic_pointer_cast<SensorType>(it->second);
            if (sensor_ptr) {
                return sensor_ptr;
            } else {
                safety::safety_procedure("SensorManager: Sensor with name %u found but is not of the requested type", static_cast<uint32_t>(name));
            }
        } else {
            safety::safety_procedure("SensorManager: Failed to get sensor with name %u, sensor not found", static_cast<uint32_t>(name));
        }
    }
private:
    /// @brief Map of sensor pointers by name
    std::map<Cfg::SensorName, std::shared_ptr<Sensor>> sensors;
};
