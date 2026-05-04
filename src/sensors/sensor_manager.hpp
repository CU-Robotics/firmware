#pragma once

#include <variant>
#include <map>
#include <memory>
#include <string>

#include "sensors/sensor.hpp"
#include "comms/config_data/robot_config.hpp"

#include "utils/safety.hpp"

#include "sensors/buff_encoder.hpp" 
#include "sensors/rev_encoder.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/LSM6DSOX.hpp"
#include "sensors/d200.hpp"
#include "sensors/limit_switch.hpp"
#include "sensors/StereoCamTrigger.hpp"


// Define all possible sensor types your manager can hold
using SensorVariant = std::variant<
    std::shared_ptr<BuffEncoder>,
    std::shared_ptr<RevEncoder>,
    std::shared_ptr<ICM20649>,
    std::shared_ptr<LSM6DSOX>,
    std::shared_ptr<D200LD14P>,
    std::shared_ptr<LimitSwitch>,
    std::shared_ptr<StereoCamTrigger>
	>;


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

	void add_sensor(Cfg::SensorName name, SensorVariant sensor);
	
    /// @brief Configure the sensors based on the configuration data
    /// @param config_data The configuration data to use to configure the sensors
    void configure_sensors(const Cfg::RobotConfig& config_data);
    /// @brief Call each sensor's init function
    void initialize_sensors();

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
    std::shared_ptr<SensorType> get_sensor_by_name(const Cfg::SensorName& name) {
        auto it = sensors.find(name);
		
        if (it != sensors.end()) {
			auto* matched_ptr = std::get_if<std::shared_ptr<SensorType>>(&it->second);
        
			if (matched_ptr && *matched_ptr) {
				return *matched_ptr;
            } else {
                safety::safety_procedure("SensorManager: Sensor with name %u found but is not of the requested type", static_cast<uint32_t>(name));
				return nullptr;
            }
        } else {
            safety::safety_procedure("SensorManager: Failed to get sensor with name %u, sensor not found", static_cast<uint32_t>(name));
			return nullptr;
        }
    }
private:
	/// @brief Map of sensor variants
	std::map<Cfg::SensorName, SensorVariant> sensors;

};
