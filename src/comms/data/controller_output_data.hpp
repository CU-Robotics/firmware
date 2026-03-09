#pragma once
#include "comms/data/comms_data.hpp"
#include "comms/config_data/motor.hpp"

struct ControllerOutputData : Comms::CommsData {
    ControllerOutputData() : CommsData(Comms::TypeLabel::ControllerOutputData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(ControllerOutputData)) { }

    float motor_outputs[MAX_NUM_MOTOR_NAMES];

    void print() const {
        printf("ControllerOutputData - motor_outputs: ");
        for (uint32_t i = 0; i < MAX_NUM_MOTOR_NAMES; i++) {
            printf("%f ", motor_outputs[i]);
        }
        printf("\n");
    }

    void set_motor_output(Cfg::MotorName motor_name, float output) {
        motor_outputs[static_cast<size_t>(motor_name)] = output;
    }
};