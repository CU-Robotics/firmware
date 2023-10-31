#ifndef ICM20649_H
#define ICM20649_H

// adafruit library specific to ICM20(...) hardware
#include <Adafruit_ICM20649.h> // https://adafruit.github.io/Adafruit_ICM20X/html/class_adafruit___i_c_m20_x.html

#include "IMUSensor.hpp"

/// @brief CS (Chip Select) pin for SPI mode
#define ICM_CS 10
/// @brief SDA/MOSI (Serial Data In) (Microcontroller Out Sensor In) pin for software-SPI mode
#define ICM_MOSI 11
/// @brief SDO/MISO (Serial Data Out) (Microcontroller In Sensor Out) pin for software-SPI mode
#define ICM_MISO 12
/// @brief SCL/SCK (SPI Clock) pin for software-SPI mode
#define ICM_SCK 13

/// @brief Sensor access for an ICM20649 IMU Sensor. Child of the abstract IMUSensor class.
/// @note supports I2C and SPI communication
class ICM20649 : public IMUSensor {
public:
    /// @brief Types of supported communication protocols supported by the ICM.
    /// @see https://learn.adafruit.com/adafruit-icm20649-wide-range-6-dof-imu-accelerometer-and-gyro/pinouts
    enum CommunicationProtocol {
        I2C, ///< Two-wire: SDA (serial data pin), SCL (serial clock pin)
        SPI ///< Serial Peripheral Interface. Four-wire: SDA (serial data in), AD0(serial data out), SCL/SCK (serial clock pin), CS (chip select pin)
    };

    /// @brief Constructor, initializes the communication protocol. Does not enable the sensor until @ref init() is called.
    /// @param protocol Which communication protocol to use for this sensor.
    explicit ICM20649(CommunicationProtocol protocol);

    /// @copydoc IMUSensor::read()    
    void read() override;
    /// @copydoc IMUSensor::init()
    void init() override;
private:
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

    /// @brief The selected communication protocol
    CommunicationProtocol protocol;
};

#endif