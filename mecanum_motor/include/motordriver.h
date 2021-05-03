#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"
#include "param.h"

class Motor
{
public:
    Motor(DigitalOut &gd, PwmOut &pwm, DigitalOut &dir, int period);
    mbed_error_status_t enableGateDriver(void);
    mbed_error_status_t disableGateDriver(void);
    void speed(float v);

protected:
    DigitalOut &GATE_DRIVER_ENABLE;
    PwmOut &_pwm;
    DigitalOut &_dir;
    int &_period;
};

#endif