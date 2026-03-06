#include "state.hpp"
#include "utils/wrapping.hpp"
#include <arduino.h>

State::State(const Cfg::State& state_config) : m_config(state_config) {
    m_state = {0, 0, 0};
}

const Cfg::State& State::config() const {
    return m_config;
}

void State::set_position(float _position) {
    if (m_config.is_wrapping) {
        _position = Utils::wrap(_position, m_config.reference_limits.position.min, m_config.reference_limits.position.max);
    } else {
        _position = constrain(_position, m_config.reference_limits.position.min, m_config.reference_limits.position.max);
    }

    m_state.position = _position;
}

float State::get_position() const {
    return m_state.position;
}

void State::set_velocity(float _velocity) {
    _velocity = constrain(_velocity, m_config.reference_limits.velocity.min, m_config.reference_limits.velocity.max);
    m_state.velocity = _velocity;
}

float State::get_velocity() const {
    return m_state.velocity;
}

void State::set_acceleration(float _acceleration) {
    _acceleration = constrain(_acceleration, m_config.reference_limits.acceleration.min, m_config.reference_limits.acceleration.max);
    m_state.acceleration = _acceleration;
}

float State::get_acceleration() const {
    return m_state.acceleration;
}

State::Raw State::get_raw() const {
    return m_state;
}

// Operator Overloads

State State::operator+(const State& other) const {
    State result(m_config);
    result.set_position(get_position() + other.get_position());
    result.set_velocity(get_velocity() + other.get_velocity());
    result.set_acceleration(get_acceleration() + other.get_acceleration());
    return result;
}

State State::operator-(const State& other) const {
    State result(m_config);
    result.set_position(get_position() - other.get_position());
    result.set_velocity(get_velocity() - other.get_velocity());
    result.set_acceleration(get_acceleration() - other.get_acceleration());
    return result;
}

State& State::operator=(const State& other) {
    if (&other == this) {
        return *this;
    }
    set_position(other.get_position());
    set_velocity(other.get_velocity());
    set_acceleration(other.get_acceleration());
    return *this;
}