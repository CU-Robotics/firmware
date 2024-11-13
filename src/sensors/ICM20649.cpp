#include "ICM20649.hpp"
#include <cassert>

// empty constructor
//ICM20649::ICM20649() {}

// initialize ICM
void ICM20649::init(CommunicationProtocol protocol) {
    // assign protocol to object.
    this->protocol = protocol;
    // begin sensor depending on selected protocol
    switch (protocol) {
    case I2C:
    {
        // start I2C communication
        if (!sensor.begin_I2C()) {
            Serial.println("Failed to begin I2C");
        }
        break;
    }
    case SPI:
    {
        // start SPI communication
        int a = sensor.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI);
        Serial.println(a);
        if (!a) {
            Serial.println("Failed to begin SPI");
        }
        break;
    }
    default:
    {
        // any other value is unexpected and will not allow the sensor to function.
        assert(false && "Invalid ICM20649 protocol selected! Expects only I2C or SPI.");
        break;
    }
    }

    // set data ranges
    // sensor.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    // sensor.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);

    // // set bit period (via divisor setting). 12-bit, Max (slowest) is 4095, min (fastest) is 0.
    // sensor.setAccelRateDivisor(0);
    // accel_rate = get_accel_data_rate();

    // // set bit period (via divisor setting). 8-bit, Max (slowest) is 255, min (fastest) is 0. 
    // sensor.setGyroRateDivisor(0);
    // gyro_rate = get_gyro_data_rate();
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

void ICM20649::set_gyro_range(int gyro_rate_range) {
    switch (gyro_rate_range) {
    default:
        sensor.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
        break;
    case (500):
        sensor.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
        break;
    case (1000):
        sensor.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
        break;
    case (2000):
        sensor.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
        break;
    case (4000):
        sensor.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
        break;
    }

    Serial.println(sensor.getGyroRange());
}

//implenet seralize and deserialize
void ICM20649::serialize(uint8_t* buffer, size_t& offset) {
    memcpy(buffer + offset, &accel.acceleration.x, sizeof(accel.acceleration.x));
    offset += sizeof(accel_X);
    memcpy(buffer + offset, &accel.acceleration.y, sizeof(accel.acceleration.y));
    offset += 4;
    memcpy(buffer + offset, &accel.acceleration.z, sizeof(accel.acceleration.z));
    offset += 4;
    memcpy(buffer + offset, &gyro.gyro.x, sizeof(gyro.gyro.x));
    offset += 4;
    memcpy(buffer + offset, &gyro.gyro.y, sizeof(gyro.gyro.y));
    offset += 4;
    memcpy(buffer + offset, &gyro.gyro.z, sizeof(gyro.gyro.z));
    offset += 4;
    memcpy(buffer + offset, &temperature, sizeof(temperature));
    offset += 4;
}

void ICM20649::deserialize(const uint8_t* data, size_t& offset) {
    memcpy(&accel_X, data + offset, sizeof(accel_X));
    offset += sizeof(accel_X);
    memcpy(&accel_Y, data + offset, sizeof(accel_Y));
    offset += sizeof(accel_Y);
    memcpy(&accel_Z, data + offset, sizeof(accel_Z));
    offset += sizeof(accel_Z);
    memcpy(&gyro_X, data + offset, sizeof(gyro_X));
    offset += sizeof(gyro_X);
    memcpy(&gyro_Y, data + offset, sizeof(gyro_Y));
    offset += sizeof(gyro_Y);
    memcpy(&gyro_Z, data + offset, sizeof(gyro_Z));
    offset += sizeof(gyro_Z);
    memcpy(&temperature, data + offset, sizeof(temperature));
    offset += sizeof(temperature);
}