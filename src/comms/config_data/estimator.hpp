#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace NewConfig {

enum class EstimatorName : uint32_t {
    UnsetEstimatorName,
    GimbalAndChassis,
    FlywheelVelocity,
    FeederPosition,
    Actuators,
};

struct LowLevelEstimator : Comms::CommsData {
    LowLevelEstimatedState estimated_states;
    EstimatorName estimator_name;

    LowLevelEstimator() : Comms::CommsData(Comms::TypeLabel::LowLevelEstimatorConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(LowLevelEstimator)) {
        estimator_name = EstimatorName::UnsetEstimatorName;
    }
};

struct HighLevelEstimator : Comms::CommsData {
    HighLevelEstimatedState estimated_states;
    EstimatorName estimator_name;

    HighLevelEstimator() : Comms::CommsData(Comms::TypeLabel::HighLevelEstimatorConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(HighLevelEstimator)) {
        estimator_name = EstimatorName::UnsetEstimatorName;
    }
};

}