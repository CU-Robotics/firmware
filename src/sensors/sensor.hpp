#pragma once
#include "comms/config_data/sensor.hpp"

class Sensor {
public: 

Sensor() = default;

virtual void init() = 0;

virtual bool read() = 0;

virtual void send_to_comms() const = 0;

}; 