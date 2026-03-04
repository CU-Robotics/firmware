#pragma once

class State {
public:
    State(const NewConfig::State& _config) : config(_config);
    const NewConfig::State& config() const;
    

    State operator+(const State& other) const;
    State operator-(const State& other) const;
    State& operator=(const State& other);
    
    void set_position(float position);
    float get_position() const;

    void set_velocity(float velocity);
    float get_velocity() const;

    void set_acceleration(float acceleration);
    float get_acceleration() const;

private:
    float position = 0;
    float velocity = 0;
    float acceleration = 0;

    const NewConfig::State& config;

private:
    inline float wrap(float value, float min, float max) const {
        float range = max - min;
        value = min + fmod(value - min, range);
        if (value < min) value += range;
        return value;
    }

    
};