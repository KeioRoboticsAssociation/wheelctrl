/*
sign機能(high speed forwardからhigh speed backしてモーターイカれるのを防ぎませう)をつけよう
dir 1が正転
*/

#include "motordriver.h"
#include "param.h"

Motor::Motor(DigitalOut &gd, PwmOut &pwm, DigitalOut &dir, int period):
    GATE_DRIVER_ENABLE(gd), _pwm(pwm), _dir(dir), _period(period) {
    GATE_DRIVER_ENABLE.write(DISABLE);
    _pwm.period_us(_period);
    _pwm.pulsewidth_us(0);
}

mbed_error_status_t Motor::enableGateDriver(void) {
    if(GATE_DRIVER_ENABLE.read() == RESET) {
        GATE_DRIVER_ENABLE.write(ENABLE);
        return MBED_SUCCESS;
    }
    else {
        MBED_ERROR(MBED_ERROR_ALREADY_INITIALIZED, "Gate Driver is already enabled");
        return MBED_ERROR_ALREADY_INITIALIZED;
    }
}

mbed_error_status_t Motor::disableGateDriver(void)
{
    if (GATE_DRIVER_ENABLE.read() == SET) {
        GATE_DRIVER_ENABLE.write(DISABLE);
        return MBED_SUCCESS;
    }
    else {
        MBED_ERROR(MBED_ERROR_ALREADY_INITIALIZED, "Gate Driver is already disabled");
        return MBED_ERROR_ALREADY_INITIALIZED;
    }
}

void Motor::speed(float v)
{
    if (v > 0){
        if(v >= _period * PWM_LIMIT) v = _period * PWM_LIMIT;
        _dir = 1;
        _pwm.pulsewidth_us(v);
    }
    else if(v < 0){
        if(v <= -_period * PWM_LIMIT) v = -_period * PWM_LIMIT;
        _dir = 0;
        _pwm.pulsewidth_us(-v);
    }
    else _pwm.pulsewidth_us(0);
}