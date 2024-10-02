#ifndef BALANCING_OBSERVER_H
#define BALANCING_OBSERVER_H

#include "../comms/rm_can.hpp"
#include "../utils/timing.hpp"
#include "../filters/IMU_Filter.hpp"
//  #include""                !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!We need to make an IMUFilter


#define L_CAN 0         // Need Check
#define R_CAN 1         // Need Check
#define L_W_MOTORID 1   // Need Check
#define L_FJ_MOTORID 2  // Need Check
#define L_BJ_MOTORID 3  // Need Check
#define R_W_MOTORID 1   // Need Check
#define R_FJ_MOTORID 2  // Need Check
#define R_BJ_MOTORID 3  // Need Check
#define l_a 0   // test
#define l_u 0   // test
#define l_l 0   // test
#define R_w 0   // test

class BalancingObserver{
    private:
        /** Helping Classes */
        Timer timer;
        /** Derivative prev variables */
        float _theta_b_old;
        float _phi_old;
        float _phi_dot_old;
        float _theta_ll_old;
        float _theta_lr_old;
        float _s_dot_old;
        float _ll_old;
        float _ll_dot_old;
        float _lr_old;
        float _lr_dot_old;
    public:
        BalancingObserver();

        void init();
        /// @brief Get the data we want for controller
        /// @param can the CANDate struct to get motor data
        /// @param imu the imu class to get IMU data
        /// @param obs the calculated number to put in the controller code
        void step(CANData* can, IMUData* imu, float obs[9][3]); 
        /// @brief print and labeled obs[9][3]
        /// @param obs the obs array we want to be print and labeled
        void testprint(float obs[9][3]);
        /// @brief Print out what sensors get and what the number after calculated
        /// @param can The can bus class
        /// @param imu the imu classes
        void settingprint(CANData* can, IMUData* imu);

};
#endif