#include "state.hpp"

State::State(const Cfg::State& state_config) : config(state_config) {
    position = 0;
    velocity = 0;
    acceleration = 0;
}

const Cfg::State& State::config() const {
    return config;
}

void State::set_position(float _position) {
    if (config.is_wrapping) {
        _position = wrap(_position, config.reference_limits.position.min, config.reference_limits.position.max);
    } else {
        _position = constrain(_position, config.reference_limits.position.min, config.reference_limits.position.max);
    }

    this->position = _position;
}

float State::get_position() const {
    return position;
}

void State::set_velocity(float _velocity) {
    _velocity = constrain(_velocity, config.reference_limits.velocity.min, config.reference_limits.velocity.max);
    this->velocity = _velocity;
}

float State::get_velocity() const {
    return velocity;
}

void State::set_acceleration(float _acceleration) {
    _acceleration = constrain(_acceleration, config.reference_limits.acceleration.min, config.reference_limits.acceleration.max);
    this->acceleration = _acceleration;
}

float State::get_acceleration() const {
    return acceleration;
}

// Operator Overloads

State State::operator+(const State& other) const {
    State result(config);
    result.set_position(this->get_position() + other.get_position());
    result.set_velocity(this->get_velocity() + other.get_velocity());
    result.set_acceleration(this->get_acceleration() + other.get_acceleration());
    return result;
}

State State::operator-(const State& other) const {
    State result(config);
    result.set_position(this->get_position() - other.get_position());
    result.set_velocity(this->get_velocity() - other.get_velocity());
    result.set_acceleration(this->get_acceleration() - other.get_acceleration());
    return result;
}

State& State::operator=(const State& other) {
    if (&other == this) {
        return *this;
    }
    this->set_position(other.get_position());
    this->set_velocity(other.get_velocity());
    this->set_acceleration(other.get_acceleration());
    return *this;
}