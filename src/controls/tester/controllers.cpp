#include "controllers.hpp"

Controllers::Controllers() {
    zero_speeds();

    can.init();
    remote.init();
}

void Controllers::update() {
    remote.read();

    // if info from the remote is not being detected
    // or if safety switch is on, don't write anything
    if (remote.get_l_switch() == 1 || remote.is_fail()) {
        Serial.println("SAFETY: ON");

        zero_speeds();
        can.zero();
        can.zero_motors();
        can.write();
    } else {
        Serial.println("SAFETY: OFF");
        while (can.read()) {}

        // here is where the main driver code will be
        // TODO: get angle of the yaw function, use that
        //       determine the context to move


        update_speeds();
        can.write();
    }

}

void Controllers::zero_speeds() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        setSpeeds[CAN_1][i] = 0;
        setSpeeds[CAN_2][i] = 0;
    }
}

void Controllers::update_speeds() {
    // currently only testing on certain moters so all of them don't freak out if the code fails
    // as for now we will just use motor one on can one
    int curSpd = can.get_motor_attribute(CAN_1, 0, SPEED);
    int dSpd = 9*(setSpeeds[CAN_1][0] - curSpd);

    int a = dSpd > 0 ? dSpd : -dSpd;

    if (a > MAX_WRITE) dSpd = dSpd / a * MAX_WRITE;

    can.write_motor(CAN_1, 0, dSpd);
}

void Controllers::set_mtr_speed(int _canNum, int _mtrNum, int val) {
    setSpeeds[_canNum][_mtrNum] = val;
}