#include "timing.h"

#ifndef STATE_H
#define STATE_H

// Maximum state length. Do not change unless you know what you're doing.
// (the Teensy and Khadas must agree on this value).
#define STATE_LEN 32

class State {
    private:
        static State* instance;

        // Here is a sample state (does not represent every robot):
        // {x, y, psi (chassis angle), theta (yaw angle), phi (pitch angle), feed, flywheel}
        // In this example case, as with all other cases, the unused state rows are kept blank.

        // State reference, also known as desired state. Stored as a concatenated 2D matrix that
        // can be split into position [0], velocity [1], and acceleration [2] vectors.
        float reference[STATE_LEN][3];

        // State estimation (from sensors), stored as a concatenated 2D matrix in the same
        // fashion as the state reference.
        float estimate[STATE_LEN][3];

        // Reference limits. Used to keep the reference physically obtainable,
        // where [n][m][0] is the low bound and [n][m][1] is the high bound.
        float reference_limits[STATE_LEN][3][2];

        // Timer for reference governor
        Timer governor_timer;

    public:
        static State* get_instance() {
            if (instance == NULL) {
                instance = new State(); 
                return instance;
            } else return instance;
        }

        void get_reference(float reference[STATE_LEN][3]);
        void set_reference(float ungoverned_reference[STATE_LEN][3]);

        void get_estimate(float estimate[STATE_LEN][3]);
        void set_estimate(float estimate[STATE_LEN][3]);
        void set_estimate_coordinates(float estimate, int row, int col);

        void set_reference_limits(float reference_limits[STATE_LEN][3][2]);
};

#endif