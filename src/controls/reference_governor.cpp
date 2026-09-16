#include "reference_governor.hpp"

#include "robot_state_array.hpp"
#include "state.hpp"
#include <memory>
#include <set>

void Governor::set_reference_array(const RobotStateArray& new_reference) {
    reference_state_array = new_reference;
}

void Governor::set_position_reference(Cfg::StateName state_name, float value) {
    reference_state_array[state_name].set_position(value);
}

void Governor::set_velocity_reference(Cfg::StateName state_name, float value) {
    reference_state_array[state_name].set_velocity(value);
}

void Governor::set_acceleration_reference(Cfg::StateName state_name, float value) {
    reference_state_array[state_name].set_acceleration(value);
}

const RobotStateArray& Governor::get_reference_array() const {
    return reference_state_array;
}

const RobotStateArray& Governor::step_reference_array(const RobotStateArray& ungoverned_reference_array) {
    float threshold = 0.0005;
    float dt = governor_timer.delta();
    
    if (count == 0) {
        dt = 0; // first dt loop generates huge time so check for that
        count++;
    }

    for (size_t i = 0; i < NUM_STATES; ++i) {
        const Cfg::StateName reference_name = static_cast<Cfg::StateName>(i);
        if (!reference_state_array.has_state(reference_name)) {
            continue;
        }
        State& reference = reference_state_array[reference_name];
        const State& ungoverned_reference = ungoverned_reference_array[reference_name];

        if (reference.config().governor_type == Cfg::StateOrder::Position) { // position based governor
            State error = ungoverned_reference.get_error_no_bounds(reference);            
            // Set the accel refrence to the max or min based on which direction it needs to go
            if (error.get_position() > threshold) reference.set_acceleration(reference.config().reference_limits.acceleration.max);
            else if (error.get_position() < -threshold) reference.set_acceleration(reference.config().reference_limits.acceleration.min);
            else {
                reference.set_acceleration(0.0);
                reference.set_velocity(ungoverned_reference.get_velocity());
            }

            if (reference.get_acceleration() != 0) {
                if (error.get_position() > 0) {
                    // check how far it will travel when braking
                    float dist_to_deccel = (pow(ungoverned_reference.get_velocity(), 2) - pow(reference.get_velocity(), 2)) 
                        / (2 * (reference.config().reference_limits.acceleration.min));

                    // if the minimum stopping distance is greater than the remaining distance start braking
                    if (dist_to_deccel > error.get_position()) {
                        if (error.get_velocity() > 0) reference.set_acceleration(reference.config().reference_limits.acceleration.max);
                        else reference.set_acceleration(reference.config().reference_limits.acceleration.min);
                    }
                } else {
                    // check how far it will travel when braking
                    float dist_to_deccel = (pow(ungoverned_reference.get_velocity(), 2) - pow(reference.get_velocity(), 2))
                         / (2 * (reference.config().reference_limits.acceleration.max));

                    // if the minimum stopping distance is greater than the remaining distance start braking
                    if (dist_to_deccel < error.get_position()) {
                        if (error.get_velocity() > 0) reference.set_acceleration(reference.config().reference_limits.acceleration.max);
                        else reference.set_acceleration(reference.config().reference_limits.acceleration.min);
                    }
                }
            }

            // step the references by higher order reference
            reference.set_velocity(reference.get_velocity() + reference.get_acceleration() * dt);
            reference.set_position(reference.get_position() + reference.get_velocity() * dt);

        } else if (reference.config().governor_type == Cfg::StateOrder::Velocity) { // velocity based governor
            State error = ungoverned_reference.get_error_no_bounds(reference);

            // check which direction the target is and set acceleration if the velocity error is less the max acceleration 
            if (error.get_velocity() > (reference.config().reference_limits.acceleration.max * dt)) {
                reference.set_acceleration(reference.config().reference_limits.acceleration.max);
            } else if (error.get_velocity() < (reference.config().reference_limits.acceleration.min * dt)) {
                reference.set_acceleration(reference.config().reference_limits.acceleration.min);
            } else {
                reference.set_velocity(ungoverned_reference.get_velocity());
                reference.set_acceleration(0);
            }
            // step the reference by the higher order reference
            reference.set_velocity(reference.get_velocity() + reference.get_acceleration() * dt);

        } else { // no governor (set the reference equal to the target)
            reference.set_acceleration(ungoverned_reference.get_acceleration());
            reference.set_velocity(ungoverned_reference.get_velocity());
            reference.set_position(ungoverned_reference.get_position());
        }
    }
    return reference_state_array;
}
