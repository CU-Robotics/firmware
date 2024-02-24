#include "state.hpp"

void State::set_reference(float reference[STATE_LEN][3]) {
    memcpy(this->reference, reference, sizeof(this->reference));
    Serial.println("Don't use this, bitch 'one time is ok :)'");
}

void State::get_reference(float reference[STATE_LEN][3]) {
    memcpy(reference, this->reference, sizeof(this->reference));
}

void State::step_reference(float ungoverned_reference[STATE_LEN][3], int governor_type[STATE_LEN]) {
    float threshold = 0.0005;
    float dt = governor_timer.delta();
    if (dt > .002)
            dt = 0; // first dt loop generates huge time so check for that
    for (int n = 0; n < STATE_LEN; n++) {
        // Keep new target values within absolute limits
        for (int p = 0; p < 3; p++) {
            if (ungoverned_reference[n][p] < reference_limits[n][p][0]) ungoverned_reference[n][p] = reference_limits[n][p][0];
            if (ungoverned_reference[n][p] > reference_limits[n][p][1]) ungoverned_reference[n][p] = reference_limits[n][p][1];
        }

        if(governor_type[n] == 1) { // position based governor
            // TODO: Iplement wrap angle 
            float pos_error = ungoverned_reference[n][0] - reference[n][0];
            float vel_error = ungoverned_reference[n][1] - reference[n][1];
            // Set the accel refrence to the max or min based on which direction it needs to go
            if(pos_error > threshold) reference[n][2] = reference_limits[n][2][1];
            else if(pos_error < -threshold) reference[n][2] = reference_limits[n][2][0];
            else {
                reference[n][2] = 0;
                reference[n][1] = ungoverned_reference[n][1];
                }

            if (reference[n][2] != 0) {
                if(pos_error > 0) {
                    // check how far it will travel when braking
                    float dist_to_deccel = (pow(ungoverned_reference[n][1],2)-pow(reference[n][1],2))/(2*(reference_limits[n][2][0]));
                   
                    // if the minimum stopping distance is greater than the remaining distance start braking
                    if (dist_to_deccel > pos_error){
                        if (vel_error > 0) reference[n][2] = reference_limits[n][2][1];
                        else reference[n][2] = reference_limits[n][2][0];
                    } 
                }
                else {
                    // check how far it will travel when braking
                    float dist_to_deccel = (pow(ungoverned_reference[n][1],2)-pow(reference[n][1],2))/(2*(reference_limits[n][2][1]));
                
                    // if the minimum stopping distance is greater than the remaining distance start braking
                    if (dist_to_deccel < pos_error){
                        if (vel_error > 0) reference[n][2] = reference_limits[n][2][1];
                        else reference[n][2] = reference_limits[n][2][0];
                    } 
                }
            }

            // step the references by higher order reference
            reference[n][1] += reference[n][2] * dt;
            reference[n][0] += reference[n][1] * dt;

        } else if(governor_type[n] == 2) { // velocity based governor
            float vel_error = ungoverned_reference[n][1] - reference[n][1];
            // check which direction the target is and set acceleration
            if(vel_error > 0) {
                if(vel_error*reference[n][1] > 0) reference[n][2] = reference_limits[n][2][1];
                else reference[n][2] = reference_limits[n][2][1];
            }
            else if(vel_error < 0) {
                if(vel_error*reference[n][1] > 0) reference[n][2] = reference_limits[n][2][0];
                else reference[n][2] = reference_limits[n][2][0];
            }
            else reference[n][2] = 0;

            // step the reference by the higher order reference
            reference[n][1] += reference[n][2] * dt;

        } else { // no governor (set the reference equal to the target)
            reference[n][2] = ungoverned_reference[n][2];
            reference[n][1] = ungoverned_reference[n][1];
            reference[n][0] = ungoverned_reference[n][0];

        }

        for (int p = 0; p < 3; p++) { 
            // Keep values within absolute limits
            if (reference[n][p] < reference_limits[n][p][0]) reference[n][p] = reference_limits[n][p][0];
            if (reference[n][p] > reference_limits[n][p][1]) reference[n][p] = reference_limits[n][p][1];
        }
    }
}

void State::get_estimate(float estimate[STATE_LEN][3]) {
    memcpy(this->estimate, estimate, sizeof(this->estimate));
}

void State::set_estimate(float estimate[STATE_LEN][3]) {
    for (int n = 0; n < STATE_LEN; n++) {
        for (int p = 0; p < 3; p++) {
            this->estimate[n][p] = estimate[n][p];
        }
    }
}

void State::set_estimate_at_location(float estimate, int row, int col) {
    if (row < 0 || row >= STATE_LEN) return; // Avoids a potential crash!
    if (col < 0 || col >= 3) return; // Avoids a potential crash!
    this->estimate[row][col] = estimate;
}

void State::set_reference_limits(float reference_limits[STATE_LEN][3][2]) {
    for (int n = 0; n < STATE_LEN; n++) {
        for (int p = 0; p < 3; p++) {
            this->reference_limits[n][p][0] = reference_limits[n][p][0];
            this->reference_limits[n][p][1] = reference_limits[n][p][1];
        }
    }
}