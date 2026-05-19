#include "ICM20649.hpp"
#include "safety.hpp"
#include "comms/data/sendable.hpp"

const SPISettings ICM20649::m_settings = SPISettings(1000000, ICM20649_BITORDER, SPI_MODE0);

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
            safety::assert_or_safety_procedure(sensor.begin_SPI(config.spi_cs, &SPI1), "ICM: Failed to begin SPI. Pins: CS: %u, SCK: %u, MISO: %u, MOSI: %u", config.spi_cs, config.spi_sck, config.spi_miso, config.spi_mosi);
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

	// ICM20649 data starts at register 0x2D (ACCEL_XOUT_H) 0x80 is the SPI read flag.
    tx_buffer[0] = 0x80 | 0x2D; 
    for(int i = 1; i < 15; i++) {
        tx_buffer[i] = 0x00; // Dummy bytes to clock out the other 14 data bytes
    }
	
    Serial.println("ICM CALIBRATING...");
    float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
    for (uint32_t i = 0; i < config.num_calibration_reads; i++) {
		if (config.communication_protocol == Cfg::CommunicationProtocol::SPI) {
            request_read();
            while (!spi_event) { yield(); }
            read();
        } else {
            read(); // I2C fallback
        }
        gyro_offset_x += get_gyro_X();
        gyro_offset_y += get_gyro_Y();
        gyro_offset_z += get_gyro_Z();
        delay(1);
    }

    gyro_offset_x /= config.num_calibration_reads;
    gyro_offset_y /= config.num_calibration_reads;
    gyro_offset_z /= config.num_calibration_reads;

    set_offsets(gyro_offset_x, gyro_offset_y, gyro_offset_z);

    Serial.printf("ICM CALIBRATION COMPLETE! Offsets: X: %f, Y: %f, Z: %f\n", gyro_offset_x, gyro_offset_y, gyro_offset_z);


}
void ICM20649::request_read() {
    if (config.communication_protocol != Cfg::CommunicationProtocol::SPI) return;
    if (transfer_in_progress)
        return;
// Configure SPI and lock the bus
    SPI1.beginTransaction(m_settings);
	
	digitalWrite(config.spi_cs, LOW); // Select the sensor (ICM is low triggerd)
    
    // Non-blocking SPI transfer
    if(!(SPI1.transfer(tx_buffer, rx_buffer, sizeof(tx_buffer), spi_event))){Serial.printf("=== Error in SPI1 transfer ===\n");}
	safety::assert_or_safety_procedure(SPI1.transfer(tx_buffer, rx_buffer, sizeof(tx_buffer), spi_event),"=== Error in SPI1 transfer ===\n");
    transfer_in_progress = true;
}

void ICM20649::read() {
	if (config.communication_protocol == Cfg::CommunicationProtocol::I2C){
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
	if (transfer_in_progress) {
        if (spi_event) {
            digitalWrite(config.spi_cs, HIGH); // Deselect the sensor

            SPI1.endTransaction();
			
			spi_event.clearEvent();
				
            transfer_in_progress = false;

            arm_dcache_delete(rx_buffer, sizeof(rx_buffer));

            // Parse the 14 bytes into the AdafruitIMUSensor protected variables
			
            // rx_buffer[0] is garbage response from setting tx_buffer[0] in init

            int16_t raw_accel_x = (rx_buffer[1] << 8)  | rx_buffer[2];
            int16_t raw_accel_y = (rx_buffer[3] << 8)  | rx_buffer[4];
            int16_t raw_accel_z = (rx_buffer[5] << 8)  | rx_buffer[6];
            
            int16_t raw_gyro_x  = (rx_buffer[7] << 8)  | rx_buffer[8];
            int16_t raw_gyro_y  = (rx_buffer[9] << 8)  | rx_buffer[10];
            int16_t raw_gyro_z  = (rx_buffer[11] << 8) | rx_buffer[12];
            
            int16_t raw_temp = (rx_buffer[13] << 8) | rx_buffer[14];

            // Apply scaling multipliers
            accel_X = raw_accel_x * accel_multiplier;
            accel_Y = raw_accel_y * accel_multiplier;
            accel_Z = raw_accel_z * accel_multiplier;

            gyro_X = raw_gyro_x * gyro_multiplier;
            gyro_Y = raw_gyro_y * gyro_multiplier;
            gyro_Z = raw_gyro_z * gyro_multiplier;
			//copy the values to the data struct
			comms_data.accel_X = accel_X;
			comms_data.accel_Y = accel_Y;
			comms_data.accel_Z = accel_Z;
			comms_data.gyro_X = gyro_X;
			comms_data.gyro_Y = gyro_Y;
			comms_data.gyro_Z = gyro_Z;
			comms_data.temperature = (raw_temp/333.87f)+21.0f; //Comfirm scaling is correct

        } else {
            // Transfer didn't finish. Retain old state.
            return; 
        }
    }
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
			accel_multiplier = GRAVITY_EARTH/8192.0;
            break;
        case Cfg::ICMImuAccelRange::A_8G:
            sensor.setAccelRange(ICM20649_ACCEL_RANGE_8_G);
			accel_multiplier = GRAVITY_EARTH/4096.0;
            break;
        case Cfg::ICMImuAccelRange::A_16G:
            sensor.setAccelRange(ICM20649_ACCEL_RANGE_16_G);
			accel_multiplier = GRAVITY_EARTH/2048.0;
            break;
        case Cfg::ICMImuAccelRange::A_30G:
            sensor.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
			accel_multiplier = GRAVITY_EARTH/1024.0;;
            break;
    }
}

void ICM20649::set_gyro_range(Cfg::ICMImuGyroRange range) {
    switch (range) {
        case Cfg::ICMImuGyroRange::DPS500:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
			gyro_multiplier = DPS_TO_RADS/65.5;
            break;
        case Cfg::ICMImuGyroRange::DPS1000:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
			gyro_multiplier = DPS_TO_RADS/32.8;
            break;
        case Cfg::ICMImuGyroRange::DPS2000:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
			gyro_multiplier = DPS_TO_RADS/16.4;
            break;
        case Cfg::ICMImuGyroRange::DPS4000:
            sensor.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
			gyro_multiplier = DPS_TO_RADS/8.2;
            break;
    }
}
