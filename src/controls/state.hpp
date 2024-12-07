#include "../utils/timing.hpp"

#ifndef STATE_H
#define STATE_H

// Maximum state length. Do not change unless you know what you're doing.
// (the Teensy and Khadas must agree on this value)
#define STATE_LEN 24

#define MICRO_STATE_LEN 3

#define NUM_ESTIMATORS 16

/// @brief Use state estimate and ungoverned reference to generated governed references to be sent to controllers.
class Governor {
private:
    // This is a sample state (it does not represent every robot):
    // {x, y, psi (chassis angle), theta (yaw angle), phi (pitch angle), feed, flywheel}
    // In this example case, as with all other cases, the unused state rows are kept blank.

    /// @brief state reference, also known as desired state. Stored as a concatenated 2D matrix that
    /// @brief can be split into position [0], velocity [1], and acceleration [2] vectors.
    float reference[STATE_LEN][3];

    /// @brief state estimation (from sensors), stored as a concatenated 2D matrix with the same
    /// @brief shape as the state reference.
    float estimate[STATE_LEN][3];

    /// @brief Reference limits. Used to keep the reference physically obtainable,
    /// @brief where [n][m][0] is the low bound and [n][m][1] is the high bound.
    float reference_limits[STATE_LEN][3][2];

    /// @brief Timer for the reference governor
    Timer governor_timer;

    /// @brief counter so dt isnt big in the first loop
    int count = 0;

public:
    /// @brief Only use one time!!!!!! Use step reference
    /// @param reference start reference at the beginning(should equal current estimate)
    void set_reference(float reference[STATE_LEN][3]);

    /// @brief Gives the instantaneous governed state reference matrix (also known as desired state)
    /// @param reference The array to override with the reference matrix; Must be of shape [STATE_LEN][3]
    void get_reference(float reference[STATE_LEN][3]);

    /// @brief Steps the reference matrix towards a goal, applying a reference governor to prevent impossible motion
    /// @param ungoverned_reference The desired robot state to step towards in the form of a matrix; Must be of shape [STATE_LEN][3]
    /// @param governor_type position based governor (1) or velocity based governor (2)
    void step_reference(float ungoverned_reference[STATE_LEN][3], const float governor_type[STATE_LEN]);

    /// @brief Gives the instantaneous state estimate matrix
    /// @param estimate The array to override with the estimate matrix; Must be of shape [STATE_LEN][3]
    void get_estimate(float estimate[STATE_LEN][3]);

    /// @brief Sets the instantaneous state estimate matrix
    /// @param estimate The current state estimate, in the form of a 2D matrix
    void set_estimate(float estimate[STATE_LEN][3]);

    /// @brief Sets the instantaneous state estimate at a specific location within the estimate matrix
    /// @param estimate Estimate value
    /// @param row The row of the matrix in which to write to; Corresponds to the index of a sub-state
    /// @param col The column of the matrix in which to write to; Corresponds to the derivative order
    void set_estimate_at_location(float estimate, int row, int col);

    /// @brief Sets the reference limits matrix which is used by the reference governor
    /// @param reference_limits Reference limits, in the form of a 3D tensor; Must be of shape [STATE_LEN][3][2]
    void set_reference_limits(const float reference_limits[STATE_LEN][3][2]);
};

#endif