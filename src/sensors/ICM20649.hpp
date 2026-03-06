#ifndef ICM20649_H
#define ICM20649_H

// adafruit library specific to ICM20(...) hardware
#include <Adafruit_ICM20649.h> 

#include "sensors/AdafruitIMUSensor.hpp"
#include "comms/data/icm_sensor_data.hpp" 


/// @brief Sensor access for an ICM20649 IMU Sensor. Child of the abstract IMUSensor class.
/// @note supports I2C and SPI communication. 
/// @see Adafruit library this class utilizes: https://adafruit.github.io/Adafruit_ICM20X/html/class_adafruit___i_c_m20_x.html
class ICM20649 : public AdafruitIMUSensor {
public:

    /// @brief Constructor. Currently does nothing, use @ref init(CommunicationProtocol) instead for initialization.
    ICM20649(const Cfg::IcmImu& config) : config(config), comms_data(config.imu_name) {}
    /// @brief Initialize the sensor with the assigned communication protocol.
    /// @param protocol Which communication protocol to use for this sensor.
    void init() override;

    /// @copydoc IMUSensor::read()    
    void read() override;

    void send_to_comms() const override;

    /// @brief set teh gyro rate range of the sensor
    /// @param gyro_rate_range new rate range
    void set_gyro_range(Cfg::ICMImuGyroRange range);
    void set_accel_range(Cfg::ICMImuAccelRange range);
private:
    const Cfg::IcmImu& config;
    /// @brief sensor object from adafruit libraries.
    Adafruit_ICM20649 sensor;

    /// @brief calculate the approximate acceleration rates in Hz from the divisor.
    /// @return acceleration data rate in Hz
    float get_accel_data_rate();
    /// @brief calculate the approximate gyroscope rates in Hz from the divisor.
    /// @return gyroscope data rate in Hz 
    float get_gyro_data_rate();

    /// @brief approximate acceleration data rate (Hz) calculated from divisor. 
    float accel_rate;

    /// @brief approximate gyroscope data rate (Hz) calculated from divisor.
    float gyro_rate;

    ///ICM sensor data.
    ICMSensorData comms_data;
};



#endif