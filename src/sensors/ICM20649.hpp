#pragma once

// adafruit library specific to ICM20(...) hardware
#include <Adafruit_ICM20649.h>
#include <SPI.h>

#include "sensors/AdafruitIMUSensor.hpp"
#include "comms/data/icm_sensor_data.hpp"


constexpr uint32_t ICM20649_BITORDER = MSBFIRST;

/// @brief Sensor access for an ICM20649 IMU Sensor. Child of the abstract IMUSensor class.
/// @note supports I2C and SPI communication. 
/// @see Adafruit library this class utilizes: https://adafruit.github.io/Adafruit_ICM20X/html/class_adafruit___i_c_m20_x.html
class ICM20649 : public AdafruitIMUSensor {
public:

    /// @brief Default constructor, initiaizes config data and comms_data with name from config.
    /// @param config configuration struct for this ICM20649 sensor
    ICM20649(const Cfg::IcmImu& config) : config(config), comms_data(config.imu_name) {}
    /// @brief Initialize the sensor with the assigned communication protocol.
    void init() override;
	/// @copydoc AdafruitIMUSensor::request_read()
	void request_read() override;
    /// @copydoc AdafruitIMUSensor::read()    
    void read() override;
    /// @brief sends the current ICM sensor data to comms
    void send_to_comms() const override;

    /// @brief set the gyro rate range of the sensor
    /// @param range new rate range
    void set_gyro_range(Cfg::ICMImuGyroRange range);
    /// @brief set the accelerometer range of the sensor
    /// @param range new rate range
    void set_accel_range(Cfg::ICMImuAccelRange range);

private:
    /// @brief Configuration data for this sensor
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

    /// @brief ICM sensor data.
    ICMSensorData comms_data;
	
	/// @brief Buffer of transmitted data to IMU
    uint8_t tx_buffer[15];
	/// @brief Buffer of recieved data from IMU
    uint8_t rx_buffer[15];

    /// @brief The Teensy object that tracks DMA completion
    EventResponder spi_event;
	/// @brief The SPI settings of the ICM IMU
    static const SPISettings m_settings;
    
    /// @brief Flag to track if there is a pending data transfer
    bool transfer_in_progress = false;
	/// @brief multiplier to adjust acceleration to m/s
    float accel_multiplier = 1.0f;
	/// @brief multiplier to adjust acceleration to rads/s
    float gyro_multiplier = 1.0f;
};
