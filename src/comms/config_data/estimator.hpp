#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace NewConfig {

enum class EstimatorName : uint8_t {
    UnsetEstimatorName,
    GimbalAndChassis,
    FlywheelVelocity,
    FeederPosition,
    Actuators,
};

struct HighLevelEstimatedState {
    int chassis_x_id;
    int chassis_y_id;
    int chassis_heading_id;
    int yaw_id;
    int pitch_id;
    int flywheel_id;
    int feeder_id;
};

struct LowLevelEstimatedState {
    int motor1_id;
    int motor2_id;
    int motor3_id;
    int motor4_id;
    int motor5_id;
    int motor6_id;
    int motor7_id;
    int motor8_id;
    int motor9_id;
    int motor10_id;
    int motor11_id;
    int motor12_id;
    int motor13_id;
    int motor14_id;
    int motor15_id;
    int motor16_id;
};

struct LowLevelEstimator : Comms::CommsData {
    LowLevelEstimatedState estimated_states;
    uint32_t estimator_name;

    LowLevelEstimator() : Comms::CommsData(Comms::TypeLabel::LowLevelEstimatorConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(LowLevelEstimator)) {
        estimator_name = UnsetEstimatorName;
    }
};

struct HighLevelEstimator : Comms::CommsData {
    HighLevelEstimatedState estimated_states;
    uint32_t estimator_name;

    HighLevelEstimator() : Comms::CommsData(Comms::TypeLabel::HighLevelEstimatorConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(HighLevelEstimator)) {
        estimator_name = UnsetEstimatorName;
    }
};

}