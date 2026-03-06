#include "ICM20649.hpp"
#include "safety.hpp"
#include "comms/data/sendable.hpp"

// initialize ICM
void ICM20649::init() {
    
    // begin sensor depending on selected protocol
    switch (config.communication_protocol) {
    case Cfg::CommunicationProtocol::I2C:
    {
        safety::assert_or_safety_procedure(sensor.begin_I2C(), "ICM: Failed to begin I2C");
        break;
    }
    case Cfg::CommunicationProtocol::SPI:
    {
        // start SPI communication. Adafruit library will handle setting the SCK, MISO, MOSI pins as outputs/inputs as needed.
        if (!sensor.begin_SPI(config.spi_cs, config.spi_sck, config.spi_miso, config.spi_mosi))
        {
            safety::assert_or_safety_procedure(sensor.begin_SPI(config.spi_cs, config.spi_sck, config.spi_miso, config.spi_mosi), "ICM: Failed to begin SPI. Pins: CS: %u, SCK: %u, MISO: %u, MOSI: %u", config.spi_cs, config.spi_sck, config.spi_miso, config.spi_mosi);
        }
        break;
    }
    default:
    {
        // any other value is unexpected and will not allow the sensor to function.
        safety::safety_procedure("Invalid ICM20649 protocol selected! Expects only I2C or SPI. Recieved: %u", static_cast<uint32_t>(config.communication_protocol));
        break;
    }
    }

    set_accel_range(config.accel_range);
    set_gyro_range(config.gyro_range);
}

void ICM20649::read() {
    // get the event data from the sensor class
    sensor.getEvent(&accel, &gyro, &temp);

    // assign result to this object's members.
    // could increase efficiency by specifying which values we need, and only assigning values to the object's members from that.
    // However, getEvent will read all values from the sensor regardless, and assigning these values is very fast

    accel_X = accel.acceleration.x;
    accel_Y = accel.acceleration.y;
    accel_Z = accel.acceleration.z;
    gyro_X = gyro.gyro.x;
    gyro_Y = gyro.gyro.y;
    gyro_Z = gyro.gyro.z;

    temperature = temp.temperature;

    //copy the values to the data struct
    comms_data.accel_X = accel_X;
    comms_data.accel_Y = accel_Y;
    comms_data.accel_Z = accel_Z;
    comms_data.gyro_X = gyro_X;
    comms_data.gyro_Y = gyro_Y;
    comms_data.gyro_Z = gyro_Z;
    comms_data.temperature = temperature;
}

void ICM20649::send_to_comms() const {
    Comms::Sendable<ICMSensorData> sendable;
    sendable.data = comms_data;
    sendable.send_to_comms();
}

// calculate the approximate acceleration data rate (Hz) from the divisor.
float ICM20649::get_accel_data_rate() {
    // equation from Adafruit ICM20649 example code
    return 1125 / (1.0 + sensor.getAccelRateDivisor());
}

// calculate the approximate gyroscope data rate from the divisor.
float ICM20649::get_gyro_data_rate() {
    // equation from Adafruit ICM20649 example code
    return gyro_rate = 1100 / (1.0 + sensor.getGyroRateDivisor());
}

void ICM20649::set_accel_range(Cfg::ICMImuAccelRange range) {
    switch (range) {
        case Cfg::ICMImuAccelRange::A_4G:
            sensor.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
            break;
        case Cfg::ICMImuAccelRange::A_8G:
            sensor.setAccelRange(ICM20649_ACCEL_RANGE_8_G);
            break;
        case Cfg::ICMImuAccelRange::A_16G:
            sensor.setAccelRange(ICM20649_ACCEL_RANGE_16_G);
            break;
        case Cfg::ICMImuAccelRange::A_30G:
            sensor.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
            break;
    }
}

void ICM20649::set_gyro_range(Cfg::ICMImuGyroRange range) {
    switch (range) {
        case Cfg::ICMImuGyroRange::DPS500:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
            break;
        case Cfg::ICMImuGyroRange::DPS1000:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
            break;
        case Cfg::ICMImuGyroRange::DPS2000:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
            break;
        case Cfg::ICMImuGyroRange::DPS4000:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
            break;
    }
}
