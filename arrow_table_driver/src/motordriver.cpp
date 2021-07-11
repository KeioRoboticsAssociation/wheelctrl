/*
sign機能(high speed forwardからhigh speed backしてモーターイカれるのを防ぎませう)をつけよう
dir 1が正転
*/

#include "motordriver.h"
#include "param.h"

Motor::Motor(PinName gd, PinName pwm_p, PinName pwm_n, float pwm_limit):
    GATE_DRIVER_ENABLE(gd), PWM_P(pwm_p), PWM_N(pwm_n), _PWM_LIMIT(pwm_limit) {
    GATE_DRIVER_ENABLE.write(DISABLE);
    PWM_P.period_us(PERIOD);
    PWM_N.period_us(PERIOD);
    PWM_P.pulsewidth_us(0);
    PWM_N.pulsewidth_us(0);
}

Motor::~Motor() {
    GATE_DRIVER_ENABLE.write(DISABLE);
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

mbed_error_status_t Motor::speed(float v)
{
    if(GATE_DRIVER_ENABLE.read() != SET) {
        MBED_ERROR(MBED_ERROR_OPERATION_PROHIBITED, "Enable Gate Driver first");
        return MBED_ERROR_OPERATION_PROHIBITED;
    }
    if (v > 0.05){
        if(v >= PERIOD * _PWM_LIMIT) v = PERIOD * _PWM_LIMIT;
        PWM_P.pulsewidth_us(v);
        PWM_N.pulsewidth_us(0);
    }
    else if(v < -0.05){
        v *= -1.0;
        if(v >= PERIOD * _PWM_LIMIT) v = PERIOD * _PWM_LIMIT;
        PWM_N.pulsewidth_us(v);
        PWM_P.pulsewidth_us(0);
    }
    else {
        PWM_P.pulsewidth_us(0);
        PWM_N.pulsewidth_us(0);
    }
    return MBED_SUCCESS;
}