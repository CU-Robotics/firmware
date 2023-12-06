#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

#include "controllers.hpp"
#include "rm_CAN.hpp"

class device_manager {
public:
    device_manager();

    // will pull the yaml file
    void init();

    // calculates each motor's torque and writes to can
    void update();

private:
    Controllers control;

    /*
        controller type array
        controller gain array
    */


private:
    void pull_yaml();

};

#endif // DEVICE_MANAGER_HPP