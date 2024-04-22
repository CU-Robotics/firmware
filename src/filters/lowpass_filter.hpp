#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

/// @brief Filter out short term fluctuations in signals
class LowpassFilter {
private:
    /// @brief gain
    float K = 0.4; // default gain
    /// @brief returned output
    float output = 0.0;

public:
    /// @brief make new lowpass filter
    /// @param K gain
    LowpassFilter(float K) { this->K = K; }
    /// @brief set gain
    /// @param K new gain
    void set_gain(float K) { this->K = K; }
    /// @brief filter and return output
    /// @param measurement input measurment
    /// @return filtered output
    float filter(float measurement) {
        output = (K * output) + ((1 - K) * measurement);
        return output;
    }
};

#endif // LOWPASS_FILTER_H