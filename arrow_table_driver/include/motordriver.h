#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"
#include "param.h"

class Motor
{
public:
    Motor(PinName gd, PinName pwm_p, PinName pwm_n, float pwm_limit);
    ~Motor();
    mbed_error_status_t enableGateDriver(void);
    mbed_error_status_t disableGateDriver(void);
    mbed_error_status_t speed(float v);

protected:
    DigitalOut GATE_DRIVER_ENABLE;
    PwmOut PWM_P;
    PwmOut PWM_N;
    float _PWM_LIMIT;
};

#endif