#pragma once

#include <Arduino.h>

#include "libraries/vn100/vn100.hpp"
#include "sensors/sensor.hpp"

class vn100 : public Sensor {
    private:
        Stream* _serial;
    public:
        vn100(Stream &serial_port);
        void init() override;
        void provide_isr_map(std::unique_ptr<RobotStateMap> *safe_map) override;
        void read() override;
        void send_to_comms() const override;
}; // docs https://docs.google.com/document/d/1nKZURsbD1O32F51RnJQU3J4d7ND6OqpvrFN9zFMxKp4/edit?tab=t.0
