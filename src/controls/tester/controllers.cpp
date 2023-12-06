#include "controllers.hpp"

Controllers::Controllers(rm_CAN* _can, DR16* _dr16) {
    zero_speeds();

    can = _can;
    remote = _dr16;
    
}

void Controllers::zero_speeds() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        setSpeeds[CAN_1][i] = 0;
        setSpeeds[CAN_2][i] = 0;
    }
}

void Controllers::update() {
    // currently only testing on certain moters so all of them don't freak out if the code fails
    // as for now we will just use motor one on can one

    int test_motor = 2; // motor 3

    int curSpd = can->get_motor_attribute(CAN_1, test_motor, SPEED);
    int dSpd = 9*(setSpeeds[CAN_1][test_motor] - curSpd);

    int a = dSpd > 0 ? dSpd : -dSpd;

    // if (a > MAX_WRITE) dSpd = dSpd / a * MAX_WRITE;

    can->write_motor(CAN_1, test_motor, dSpd);
}

void Controllers::set_mtr_speed(int _canNum, int _mtrNum, int val) {
    setSpeeds[_canNum][_mtrNum] = val;
}