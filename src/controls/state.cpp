#include "state.hpp"

float* State::get_reference() {
    /*
    Provides the instantaneous governed state reference 2D matrix (also known as desired state).
    @param
        None
    @return
        reference: (float*) Desired robot state, in the form of a matrix.
    */
    return reference;
}

void State::set_reference(float ungoverned_reference[STATE_LEN][3]) {
    /*
    Sets the reference matrix, applying a reference governor to prevent impossible motion.
    This function acts as a step, and therefore must be called at the same frequency as controllers.
    @param
        ungoverned_reference: (float*) Desired robot state, in the form of a 2D matrix.
    @return
        None
    */
    float dt = governor_timer.delta();

    for (int n = 0; n < STATE_LEN; n++) {
        // Keep values within absolute limits
        for (int p = 0; p < 3; p++) {
            if (reference[n][p] < reference_limits[n][p][0]) reference[n][p] = reference_limits[n][p][0];
            if (reference[n][p] > reference_limits[n][p][1]) reference[n][p] = reference_limits[n][p][1];
        }

        // Step position and velocity towards ungoverned reference
        for (int p = 0; p < 2; p++) {
            float error = ungoverned_reference[n][p] - reference[n][p];
            if (error < reference_limits[n][p+1][0] * dt) reference[n][p] -= reference_limits[n][p+1][0] * dt;
            else if (error > reference_limits[n][p+1][1] * dt) reference[n][p] += reference_limits[n][p+1][1] * dt;
            else reference[n][p] = ungoverned_reference[n][p]; // This happens if getting to the ungoverned reference is achievable in this timestep
        }
        
        // Acceleration always jumps directly to ungoverned reference
        reference[n][3] = ungoverned_reference[n][3];

        // Keep values within absolute limits
        for (int p = 0; p < 3; p++) {
            if (reference[n][p] < reference_limits[n][p][0]) reference[n][p] = reference_limits[n][p][0];
            if (reference[n][p] > reference_limits[n][p][1]) reference[n][p] = reference_limits[n][p][1];
        }
    }
}

float* State::get_estimate() {
    /*
    Provides the instantaneous state estimate 2D matrix.
    @param
        None
    @return
        estimate: (float*) State estimate, in the form of a 2D matrix.
    */
    return estimate;
}

void State::set_estimate(float estimate[STATE_LEN][3]) {
    /*
    Sets the instantaneous state estimate 2D matrix.
    @param
        estimate: (float*) State estimate, in the form of a 2D matrix.
    @return
        None
    */
    for (int n = 0; n < STATE_LEN; n++) {
        for (int p = 0; p < 3; p++) {
            this.estimate[n][p] = estimate[n][p];
        }
    }
}

void State::set_estimate_row(float estimate[3], int row) {
    /*
    Sets a single row of instantaneous state estimate 2D matrix.
    @param
        estimate: (float*) State estimate, in the form of a vactor (pos, vel, accel).
        row: (int) The row of the matrix in which to write to. Corresponds to the index of a particular state value.
    @return
        None
    */
    if (row < 0 || row >= STATE_LEN) return; // Avoids a potential crash!
    for (int p = 0; p < 3; p++) this.estimate[row][p] = estimate[p];
}

void State::set_reference_limits(float reference_limits[STATE_LEN][3][2]) {
    /*
    Sets the reference limits estimate matrix.
    @param
        reference_limits: (float*) Reference limits, in the form of a 3D matrix.
    @return
        None
    */
    for (int n = 0; n < STATE_LEN; n++) {
        for (int p = 0; p < 3; p++) {
            this.reference_limits[n][p] = reference_limits[n][p];
        }
    }
}