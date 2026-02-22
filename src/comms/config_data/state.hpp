#pragma once

namespace NewConfig {

enum class StateName : uint32_t {
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    Flywheels,
    Feeder,
};

enum class StateOrder : uint32_t {
    Position,
    Velocity,
    Acceleration,
}

struct StateIndices {
    uint32_t chassis_x_id;
    uint32_t chassis_y_id;
    uint32_t chassis_heading_id;
    uint32_t yaw_id;
    uint32_t pitch_id;
    uint32_t flywheel_id;
    uint32_t feeder_id;
};

struct State : Comms::CommsData {
    StateIndices state_indices;

    State() : Comms::CommsData(Comms::TypeLabel::StateConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(State)) {
        state_indices = StateIndices();
    }

    std::optional<size_t> state_enum_to_array_index(StateName state_name) const {
        switch (state_name) {
            case StateName::ChassisX:
                return static_cast<size_t>(state_indices.chassis_x_id);
            case StateName::ChassisY:
                return static_cast<size_t>(state_indices.chassis_y_id);
            case StateName::ChassisHeading:
                return static_cast<size_t>(state_indices.chassis_heading_id);
            case StateName::GimbalYaw:
                return static_cast<size_t>(state_indices.yaw_id);
            case StateName::GimbalPitch:
                return static_cast<size_t>(state_indices.pitch_id);
            case StateName::Flywheels:
                return static_cast<size_t>(state_indices.flywheel_id);
            case StateName::Feeder:
                return static_cast<size_t>(state_indices.feeder_id);
            default:
                Serial.printf("STATE CONFIG CRITICAL: Requested index of an invalid state name %u! Returning nullopt\n", static_cast<uint32_t>(state_name));
                return std::nullopt;
        }
    }

    std::optional<size_t> order_enum_to_array_index(StateOrder state_order) const {
        switch (state_order) {
            case StateOrder::Position:
                return 0;
            case StateOrder::Velocity:
                return 1;
            case StateOrder::Acceleration:
                return 2;
            default:
                Serial.printf("STATE CONFIG CRITICAL: Requested index of an invalid state order %u! Returning nullopt\n", static_cast<uint32_t>(state_order));
                return std::nullopt;
        }

    }
};

};