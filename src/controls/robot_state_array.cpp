#include "robot_state_array.hpp"
#include "utils/safety.hpp"
#include <cstddef>
#include <cstring>

RobotStateArray::RobotStateArray(const std::vector<Cfg::State>& _state_configurations) {
    for (const auto& state_cfg : _state_configurations) {
        if (state_cfg.name == Cfg::StateName::UnsetStateName || state_cfg.name == Cfg::StateName::StateNameCount) {
            safety::safety_procedure("RobotStateArray: Invalid state name in state configurations: %u", static_cast<uint32_t>(state_cfg.name));
        }
        const size_t idx = static_cast<size_t>(state_cfg.name);
        if (idx < NUM_STATES) {
            robot_states[idx].emplace(state_cfg);
        }
    }
}

State& RobotStateArray::operator[](Cfg::StateName state_name) {
    const size_t idx = static_cast<size_t>(state_name);
    safety::assert_or_safety_procedure(
        idx < NUM_STATES && robot_states[idx].has_value(),
        "RobotStateArray: Requested state name %u is uninitialized or out of bounds.",
        static_cast<uint32_t>(state_name)
    );
    return *robot_states[idx];
}

const State& RobotStateArray::operator[](Cfg::StateName state_name) const {
    const size_t idx = static_cast<size_t>(state_name);
    safety::assert_or_safety_procedure(
        idx < NUM_STATES && robot_states[idx].has_value(),
        "RobotStateArray: Requested state name %u is uninitialized or out of bounds.",
        static_cast<uint32_t>(state_name)
    );
    return *robot_states[idx];
}
void RobotStateArray::from_comms_packet(const State::Raw incoming_states[NUM_STATES]) {
    for (size_t i = 0; i < NUM_STATES; i++) {
        if (robot_states[i].has_value()) {
            robot_states[i]->set_position(incoming_states[i].position);
            robot_states[i]->set_velocity(incoming_states[i].velocity);
            robot_states[i]->set_acceleration(incoming_states[i].acceleration);
        }
    }
}

void RobotStateArray::print() const{
    Serial.println("RobotStateArray:");

    auto state_to_str = [](Cfg::StateName name) -> const char* {
        switch (name) {
            case Cfg::StateName::UnsetStateName: return "UnsetStateName";
            case Cfg::StateName::ChassisX:       return "X";
            case Cfg::StateName::ChassisY:       return "Y";
            case Cfg::StateName::ChassisHeading: return "Z";
            case Cfg::StateName::GimbalYaw:      return "YAW";
            case Cfg::StateName::GimbalPitch:    return "PITCH";
            case Cfg::StateName::Flywheels:      return "Flywheels";
            case Cfg::StateName::Feeder:         return "Feeder";
            case Cfg::StateName::LowerFeeder:    return "LowerFeeder";
            case Cfg::StateName::StructPadding:  return "StructPadding";
            case Cfg::StateName::StateNameCount: return "StateNameCount";
            default:                             return "UNKNOWN";
        }
    };

    for (size_t i = 0; i < NUM_STATES; i++) {
        if (robot_states[i].has_value()) {
            const auto& state = *robot_states[i];
            const Cfg::StateName state_name = static_cast<Cfg::StateName>(i);

            Serial.printf("\tState: %-12s | Pos: %8.3f | Vel: %8.3f | Acc: %8.3f\n",
                state_to_str(state_name),
                state.get_position(),
                state.get_velocity(),
                state.get_acceleration());

            Serial.printf("\t\tPos Limits: [%.2f, %.2f]\n",
                state.config().reference_limits.position.min,
                state.config().reference_limits.position.max);
        }
    }
}
