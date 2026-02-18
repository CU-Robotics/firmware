#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "ACS712.hpp"
#include "Sensor.hpp"
#include "StereoCamTrigger.hpp"
#include "config_layer.hpp"
#include "d200.hpp"
#include "ICM20649.hpp"
#include <SPI.h>
#include "buff_encoder.hpp"
#include "TOFSensor.hpp"
#include "rev_encoder.hpp"
#include "sensors/d200.hpp"
#include <Arduino.h>
#include "RefSystem.hpp"
#include "comms/data/sendable.hpp"
#include "sensors/limit_switch.hpp"


#define NUM_SENSOR_TYPE 16
#define NUM_IMU_CALIBRATION 50000

/// @class SensorManager
/// @brief Class to manage sensors on the robot
class SensorManager {
public:

    /// @brief Constructor for the SensorManager class
    SensorManager();

/// @brief Destructor for the SensorManager class
    ~SensorManager();

    /// @brief Initialize the sensor manager with configuration data
    void init();

    void configure(const NewConfig::RobotConfig& config_data);

    /// @brief Read all sensor data and send to comms
    void read_and_send_to_comms();

    std::optional<BuffEncoder*> get_buff_encoder_by_name(NewConfig::BuffEncoderName name);
    std::optional<RevEncoder*> get_rev_encoder_by_name(NewConfig::RevEncoderName name);
    std::optional<ICM20649*> get_icm_sensor_by_name(NewConfig::ImuName name);
    std::optional<LSM6DSOX*> get_lsm_sensor_by_name(NewConfig::ImuName name);
    std::optional<D200LD14P*> get_lidar_sensor_by_name(NewConfig::D200LidarName name);
    std::optional<LimitSwitch*> get_limit_switch_by_name(NewConfig::LimitSwitchName name);
    std::optional<ACS712*> get_current_sensor_by_name(NewConfig::CurrentSensorName name);
    std::optional<TOFSensor*> get_tof_sensor_by_name(NewConfig::TOFSensorName name);
    std::optional<StereoCamTrigger*> get_stereo_cam_trigger_by_name(NewConfig::StereoCameraTriggerName name);


    /// @brief Get the referee system
    /// @return pointer to the referee system
    RefSystem* get_ref() {
        return ref;
    }

    /// @brief Set the estimated state of the robot
    /// @param estimated_state array of the estimated state
    void set_estimated_state(float estimated_state[STATE_LEN][3]) {
        memcpy(this->estimated_state, estimated_state, sizeof(this->estimated_state));
    }

private:
    /// @brief Array to store the estimated state of the robot, used by sensors that adjust from the estimated state
    float estimated_state[STATE_LEN][3] = { {0} };

    // Unfortunately, sensors and their configurations are too different to group into a shared parent with a shared config, 
    // so we separately store sensors of each type in a map with their corresponding sendable objects for comms.
    
    std::map<BuffEncoder, Comms::Sendable<BuffEncoderData>> buff_encoders;
    std::map<RevEncoder, Comms::Sendable<RevSensorData>> rev_encoders;
    std::map<ICM20649, Comms::Sendable<ICMSensorData>> icm_imus;
    std::map<LSM6DSOX, Comms::Sendable<LSMSensorData>> lsm_imus;
    std::map<D200LD14P, Comms::Sendable<LidarDataPacketSI>> d200_lidars;
    std::map<LimitSwitch, Comms::Sendable<LimitSwitchData>> limit_switches;
    std::map<ACS712, Comms::Sendable<ACS712Data>> acs712_current_sensors;
    std::map<TOFSensor, Comms::Sendable<TOFSensorData>> tof_sensors;
    std::map<StereoCamTrigger, Comms::Sendable<StereoCamTriggerData>> stereo_cam_triggers;
};