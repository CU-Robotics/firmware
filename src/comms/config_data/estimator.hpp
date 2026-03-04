#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace NewConfig {

enum class EstimatorType : uint32_t {
    UnsetEstimatorType,
    GimbalAndChassis,
    FlywheelVelocity,
    FeederPosition,
    Actuators,
};

struct HighLevelEstimator : Comms::CommsData {
    EstimatorType estimator_type;
    StateName state_name[STATE_LEN];

    HighLevelEstimator() : Comms::CommsData(Comms::TypeLabel::HighLevelEstimatorConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(HighLevelEstimator)) {
        estimator_type = EstimatorType::UnsetEstimatorType;
    }
};

}