#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"

class Motor
{
public:
    Motor(PwmOut &pwm, DigitalOut &dir, double period);
    void speed(double v);

protected:
    PwmOut &_pwm;
    DigitalOut &_dir;
    double &_period;
};

#endif