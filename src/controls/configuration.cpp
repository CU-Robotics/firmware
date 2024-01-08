#include "configuration.hpp"

void Configuration::get_kinematics(float _kinematics[2][NUM_MOTORS][STATE_LEN]){
    std::memcpy(kinematics, _kinematics, sizeof(kinematics));
}

void Configuration::get_controller_weights(float _controller_weights[NUM_MOTORS][WEIGHTS_LEN]){
    std::memcpy(controller_weights, _controller_weights, sizeof(controller_weights));
}

void Configuration::get_controller_types(int _controller_types[NUM_MOTORS]){
    std::memcpy(controller_types, _controller_types, sizeof(controller_types));
}

void Configuration::set_kinematics(float _kinematics[2][NUM_MOTORS][STATE_LEN], int size){
    std::memcpy(_kinematics, kinematics, size);
}

void Configuration::set_controller_weights(float _controller_weights[NUM_MOTORS][WEIGHTS_LEN], int size){
    std::memcpy(_controller_weights, controller_weights, size);
}

void Configuration::set_controller_types(int _controller_types[NUM_MOTORS], int size){
    std::memcpy(_controller_types, controller_types, size);
}