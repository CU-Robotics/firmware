#pragma once
#include "comms/config_data/state.hpp"

class State {
    public: 
        struct Raw {
            float position;
            float velocity;
            float acceleration;
        };

    public:
        State(const Cfg::State& _config);
        const Cfg::State& config() const;
        
        State operator+(const State& other) const;
        State operator-(const State& other) const;
        State& operator=(const State& other);
        
        void set_position(float position);
        float get_position() const;

        void set_velocity(float velocity);
        float get_velocity() const;

        void set_acceleration(float acceleration);
        float get_acceleration() const;

        Raw get_raw() const;

    private:
        Raw m_state;

        const Cfg::State& m_config;
};