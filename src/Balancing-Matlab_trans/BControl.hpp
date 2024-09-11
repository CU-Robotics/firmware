#ifndef BALANCING_CONTROL_H
#define BALANCING_CONTROL_H
#include "../comms/rm_can.hpp"
#include "../filters/pid_filter.hpp"

/** Constant */
#define param.m_b 
#define param.m_l 
#define param.R_l 
#define led_controller.eta_l
#define p 

class Balacing_control{
    private:
    /** Variables */
        float _x_d;
        float _psi_d;
        float _l_d;
        float _x;
        float _psi;
        float _ll;
        float _lr;
        float _jl;
        float _jr;
        float _a_z;

    /** Helping Functions */

    public:
    /** Access Functions */
        void write_input_fdb(float x, float psi, float ll, float lr, float jl, float jr, float a_z);
        void write_input_ref(float x_d, float psi_d, float l_d);
        
};
#endif