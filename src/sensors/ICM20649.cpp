#include "ICM20649.hpp"
#include <cassert>

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
            Serial.println("ICM: Failed to begin I2C");
        }
        break;
    }
    case SPI:
    {
        // start SPI communication
        if (!sensor.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI))
        {
            Serial.println("ICM: Failed to begin SPI");
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

bool ICM20649::read() {
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
    icm_sensor_data.accel_X = accel_X;
    icm_sensor_data.accel_Y = accel_Y;
    icm_sensor_data.accel_Z = accel_Z;
    icm_sensor_data.gyro_X = gyro_X;
    icm_sensor_data.gyro_Y = gyro_Y;
    icm_sensor_data.gyro_Z = gyro_Z;
    icm_sensor_data.temperature = temperature;
    return true;
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
}
