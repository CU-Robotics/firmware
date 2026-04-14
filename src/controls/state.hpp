#pragma once
#include "comms/config_data/state.hpp"

/// @brief The State class represents the state of a particular joint of the robot
class State {
    public: 
        /// @brief Raw state struct for direct access to state values.
        struct Raw {
            /// @brief The position of the state.
            float position;
            /// @brief The velocity of the state.
            float velocity;
            /// @brief The acceleration of the state.
            float acceleration;
        };

    public:
        /// @brief Construct a new State object with the given configuration.
        /// @param _config The configuration for the state; this includes the name of the state, the limits of the state, and whether the state wraps at those limits.
        State(const Cfg::State& _config);

        /// @brief Get the configuration for this state.
        /// @return A const reference to the configuration for this state.
        const Cfg::State& config() const;
        
        
        /// @brief Set the position value of the state.
        /// @param position The position value to set. This will be constrained (or wrapped if is_wrapping is true) to the limits specified in the configuration.
        void set_position(float position);

        
        /// @brief Get the position value of the state.
        /// @return The position value of the state.
        float get_position() const;
        
        /// @brief Set the velocity value of the state.
        /// @param velocity The velocity value to set. This will be constrained to the limits specified in the configuration.
        void set_velocity(float velocity);
        
        /// @brief Get the velocity value of the state.
        /// @return The velocity value of the state.
        float get_velocity() const;
        
        /// @brief Set the acceleration value of the state.
        /// @param acceleration The acceleration value to set. This will be constrained to the limits specified in the configuration.
        void set_acceleration(float acceleration);
        
        /// @brief Get the acceleration value of the state.
        /// @return The acceleration value of the state.
        float get_acceleration() const;
        
        /// @brief Get the difference between this state and another state, ignoring limits. 
        // This will simply subtract the position, velocity, and acceleration values of the two states without applying any constraints or wrapping.
        /// @param other The other state to compare to this state.
        /// @return A new State object that represents the error between this state and the other state, without applying limits or wrapping.
        State get_error_no_bounds(const State& other) const;
        
        /// @note These arithmetic operators are overloaded so that the state limits are preserved when doing arithmetic operations on states.

        /// @brief Operator+: Add two states together, preserving limits and wrapping if necessary.
        /// @param other The other state to add to this state.
        /// @return The resulting sum state.
        State operator+(const State& other) const;
        
        /// @brief Operator-: Subtract one state from another, preserving limits and wrapping if necessary.
        /// @param other The other state to subtract from this state.
        /// @return The resulting difference state.
        State operator-(const State& other) const;
        
        /// @note The assignment operator is overloaded so that the configuration of the state is preserved when assigning one state to another.
        
        /// @brief Operator=: Assign the value of another state to this state, preserving the original configuration of this state.
        /// @param other The other state to assign to this state.
        /// @return A reference to this state.
        State& operator=(const State& other);
        
        /// @brief Get a copy of the raw state values.
        /// @return A Raw struct containing the position, velocity, and acceleration values of this state.
        Raw get_raw() const;
        
        private:
        /// @brief The raw state values. This is used for direct access to the state values, and is updated whenever the state is updated.
        Raw m_state;
        
        /// @brief The configuration for the state; this includes the name of the state, the limits of the state, and whether the state wraps at those limits.
        const Cfg::State& m_config;
    private:
        /// These functions set state values without applying limits or wrapping. 
        // They should only be used in specific cases, such as when calculating the error between two states, 
        // where we want to preserve the actual difference between the states without having it be affected by limits or wrapping.
        
        /// @brief Set the position value of the state without applying limits or wrapping.
        /// @param position The position value to set.
        void set_position_no_bound(float position);
        /// @brief Set the velocity value of the state without applying limits.
        /// @param velocity The velocity value to set.
        void set_velocity_no_bound(float velocity);
        /// @brief Set the acceleration value of the state without applying limits.
        /// @param acceleration The acceleration value to set.
        void set_acceleration_no_bound(float acceleration);
    };