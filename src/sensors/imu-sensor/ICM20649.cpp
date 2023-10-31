#include "ICM20649.hpp"
#include <cassert>

// constructor
ICM20649::ICM20649(CommunicationProtocol protocol) {
    // assign protocol to object.
    this->protocol = protocol
}

// initialize ICM
void ICM20649::init() {
    // begin sensor depending on selected protocol
    switch (protocol) {
    case I2C:
        // start I2C communication 
        if (!sensor.begin_I2C()) {
            Serial.println("Failed to begin I2C");
        }
        break;
    case SPI:
        // start SPI communication
        if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
            Serial.println("Failed to begin SPI")
        }
        break;
    default:
        // any other value is unexpected and will not allow the sensor to function.
        assert(false && "Invalid ICM20649 protocol selected! Expects only I2C or SPI.")
        break;
    }

    // set data ranges
    sensor.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    sensor.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);

    // set accel rate (via divisor setting)
    sensor.setAccelRateDivisor(4095);
    accel_rate = get_accel_data_rate();

    // set gyro rate (via divisor setting)
    sensor.setGyroRateDivisor(255);
    gyro_rate = get_gyro_data_rate();
}

void ICM20649::read() {
    // get the event data from the sensor class
    sensor.getEvent(&accel, &gyro, &temp);

    // assign result to this object's members.
    // (could increase efficiency by specifying which values we need, and only assigning values to the object's members from that. However, getEvent will read all values from the sensor regardless)
    accel_X = accel.acceleration.x;
    accel_Y = accel.acceleration.y;
    accel_Z = accel.acceleration.z;
    gyro_X = gyro.gyro.x;
    gyro_Y = gyro.gyro.y;
    gyro_Z = gyro.gyro.z;

    temperature = temp.temperature;
}

// calculate the approximate acceleration rate (Hz) from the divisor.
float ICM20649::get_accel_data_rate() {
    // equation from Adafruit ICM20649 example code
    return 1125 / (1.0 + sensor.getAccelRateDivisor());
}

float ICM20649::get_gyro_data_rate() {
    // equation from Adafruit ICM20649 example code
    return gyro_rate = 1100 / (1.0 + sensor.getGyroRateDivisor());
}