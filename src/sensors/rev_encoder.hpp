#pragma once

#include <FreqMeasureMulti.h>
#include "sensors/sensor.hpp"
#include "comms/data/rev_encoder_data.hpp"

/// @brief the class for the Rev Through Bore Encoder(www.revrobotics.com/rev-11-1271/)
class RevEncoder : public Sensor{
private:
	/// @brief configuration struct for this rev encoder
	const Cfg::RevEncoder& config;
	/// @brief Used to read rise time of the encoder
	FreqMeasureMulti freq;
	/// @brief measure of current angle in ticks [0, 1023]
	int ticks;
	/// @brief measure of current angle in radians [0, 2pi)
	float radians;
	/// @brief the starting value of the encoder in radians
	float starting_value = 0;
	/// @brief data to be sent to comms
	RevSensorData comms_data;
public:
	/// @brief Construct a new rev_encoder object
	/// @param config the configuration struct for this rev encoder, containing the encoder name and encoder pin
	RevEncoder(const Cfg::RevEncoder& config) : Sensor(), config(config), comms_data(config.encoder_name) {};

	/// @brief initialize the encoder
	void init() override;

	/// @brief updates ticks and radians to the current angle 
	void read() override;
	/// @brief sends the current rev sensor data to comms
	void send_to_comms() const override;
	
	/// @brief get the last angle of the encoder in ticks
	/// @return the last angle of the encoder in ticks [0, 1023]
	float get_angle_ticks();
	/// @brief get the last angle of the encoder in radians
	/// @return the last angle of the encoder in radians [0, 2pi)
	float get_angle_radians();

	/// @brief print the encoder details
	void print();
	/// @brief Prints a formatted dashboard of live Rev Encoder values
	void print_live_data() override;
};
