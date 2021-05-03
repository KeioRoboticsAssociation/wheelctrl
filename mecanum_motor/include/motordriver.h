#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"
#include "param.h"

class Motor
{
public:
    Motor(PwmOut &pwm, DigitalOut &dir, int period);
    void speed(float v);

protected:
    PwmOut &_pwm;
    DigitalOut &_dir;
    int &_period;
};

#endif