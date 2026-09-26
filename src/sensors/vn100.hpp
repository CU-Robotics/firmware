#pragma once

#include <Arduino.h>

#include "sensors/sensor.hpp"

class vn100 : public Sensor {
    private:

    public:
        void init() override;
        void provide_isr_map(std::unique_ptr<RobotStateMap> *safe_map) override;
        void read() override;
        void send_to_comms() const override;
}; // docs https://docs.google.com/document/d/1nKZURsbD1O32F51RnJQU3J4d7ND6OqpvrFN9zFMxKp4/edit?tab=t.0