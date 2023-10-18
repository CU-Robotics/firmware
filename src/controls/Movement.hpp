#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include <iostream>

class Movement {
    public:
        Movement();
        void move(double desiredDirection, double desiredVelocity);

    private:
        double flVel;
        double frVel;
        double blVel;
        double brVel;
};

#endif
