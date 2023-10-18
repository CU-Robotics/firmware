#include <iostream>
#include "Movement.hpp"
#include <cmath>

Movement::Movement() {
    flVel = 0;
    frVel = 0;
    blVel = 0;
    brVel = 0;
}

void Movement::move(double desiredDirection, double desiredVelocity) {
    double pi = 2 * acos(0.0); 

    double P1 = -1 * cos(desiredDirection + (pi / 4));
    double P2 = sin(desiredDirection + (pi / 4));
    double s = std::max(abs(P1), abs(P2)) / desiredVelocity;

    flVel = P2 / s;
    frVel = P1 / s;
    blVel = P1 / s;
    brVel = P2 / s;
}