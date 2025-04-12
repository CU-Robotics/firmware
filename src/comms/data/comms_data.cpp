#include "comms_data.hpp"

#if defined(HIVE)
#include "doctest/doctest.h"    // for doctest

TEST_CASE("To string does stuff") {
    CHECK(Comms::to_string(Comms::TypeLabel::NONE) == "NONE");
    CHECK(Comms::to_string(Comms::TypeLabel::TestData) == "TestData");
    CHECK(Comms::to_string(Comms::TypeLabel::BigTestData) == "BigTestData");
    CHECK(Comms::to_string(Comms::TypeLabel::LoggingData) == "LoggingData");
    CHECK(Comms::to_string(Comms::TypeLabel::BuffEncoderData) == "BuffEncoderData");
    CHECK(Comms::to_string(Comms::TypeLabel::ICMSensorData) == "ICMSensorData");
    CHECK(Comms::to_string(Comms::TypeLabel::RevEncoderData) == "RevEncoderData");
    CHECK(Comms::to_string(Comms::TypeLabel::TOFSensorData) == "TOFSensorData");
    CHECK(Comms::to_string(Comms::TypeLabel::LidarDataPacketSI) == "LidarDataPacketSI");
    CHECK(Comms::to_string(Comms::TypeLabel::DR16Data) == "DR16Data");
    CHECK(Comms::to_string(Comms::TypeLabel::TempRobotState) == "TempRobotState");
    CHECK(Comms::to_string(Comms::TypeLabel::TargetState) == "TargetState");
    CHECK(Comms::to_string(Comms::TypeLabel::EstimatedState) == "EstimatedState");
    CHECK(Comms::to_string(Comms::TypeLabel::OverrideState) == "OverrideState");
    CHECK(Comms::to_string(Comms::TypeLabel::ConfigSection) == "ConfigSection");
    CHECK(Comms::to_string(Comms::TypeLabel::CommsRefData) == "CommsRefData");
    CHECK(Comms::to_string(Comms::TypeLabel(0xff)) == "UNKNOWN");
}

#endif