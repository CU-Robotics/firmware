#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"

#ifndef CONTROLLER_H
#define CONTROLLER_H

#define NUM_GAINS 12

struct Controller {
    protected:
        float gains[NUM_GAINS];
        Timer timer;

    public:
        Controller(){};

        void set_gains(float _gains[NUM_GAINS]){
            for(int i = 0; i < NUM_GAINS; i++)
                gains[i] = _gains[i];
        }

        /// @brief Generates a motor current from a joint reference and estimation
        /// @returns motor output
        virtual float step(float reference[3], float estimate[3]);

        /// @brief Resets integrators/timers
        virtual void reset() { timer.start_timer(); }
};

struct NullController : public Controller {
    public:
        float step(float reference[3], float estimate[3]) {return 0;}
};

struct PIDPositionController : public Controller {
    private:
        PIDFilter pid;

    public:

        float step(float reference[3], float estimate[3]) {
            float dt = timer.delta();

            pid.setpoint = reference[0]; // 0th index = position
            pid.measurement = estimate[0];

            float output = pid.filter(dt);
            return output;
        }

        void reset() {
            Controller::reset();
            pid.sumError = 0.0;
        }
};

struct PIDVelocityController : public Controller {
    private:
        PIDFilter pid;


    public:
        float step(float reference[3], float estimate[3]) {
            float dt = timer.delta();

            pid.setpoint = reference[1]; // 1st index = position
            pid.measurement = estimate[1];
            pid.K[0] = gains[0];
            pid.K[1] = gains[1];
            pid.K[2] = gains[2];

            float output = pid.filter(dt);
            return output;
        }

        void reset() {
            Controller::reset();
            pid.sumError = 0.0;
        }
};

struct FullStateFeedbackController : public Controller {
    private:
        PIDFilter pid1, pid2;
      
      public:
          void set_gains(float gains[NUM_GAINS]) {
              Controller::set_gains(gains);
            //   memcpy(gains[0], pid1.K, sizeof(pid1.K));
            //   memcpy(gains[3], pid2.K, sizeof(pid2.K));
          }

          float step(float reference[3], float estimate[3]) {
              float dt = timer.delta();
              float output = 0.0;

              pid1.setpoint = reference[0];
              pid1.measurement = estimate[0];

              pid2.setpoint = reference[1];
              pid2.measurement = estimate[1];

              output += pid1.filter(dt);
              output += pid2.filter(dt);
              
              return output;
          }

          void reset() {
              Controller::reset();
              pid1.sumError = 0.0;
              pid2.sumError = 0.0;
          }
};


#endif // CONTROLLER_H