#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

/// @brief Filter out short term fluctuations in signals
class LowpassFilter {
    private:
        float K = 0.4; // default gain
        float output = 0.0;

    public:
        LowpassFilter(float K) { this->K = K; }
        void set_gain(float K) { this->K = K; }
        float filter(float measurement) {
            output = (K * output) + ((1 - K) * measurement);
            return output;
        }
};

#endif // LOWPASS_FILTER_H