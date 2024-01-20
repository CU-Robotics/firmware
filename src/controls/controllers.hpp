#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#define NUM_GAINS 12

struct Controller {
    private:
        float gains[NUM_GAINS];
    public:
        Controller();

        /// @brief Sets this controller's gains
        void set_gains(float gains[NUM_GAINS]) { memcpy(gains, this->gains, NUM_GAINS) }

        /// @brief Generates a motor current from a joint reference and estimation
        /// @returns motor output
        float step(float reference[3], float estimate[3]);
}

struct NullController : public Controller {
    public:
        float step(float reference[3], float estimate[3]) { return 0; }
}

#endif // CONTROLLERS_H