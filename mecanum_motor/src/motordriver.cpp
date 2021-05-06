/*
sign機能(high speed forwardからhigh speed backしてモーターイカれるのを防ぎませう)をつけよう
dir 1が正転
*/

#include "motordriver.h"
#include "param.h"

Motor::Motor(PinName gd, PinName pwm_p, PinName pwm_n):
    GATE_DRIVER_ENABLE(gd), PWM_P(pwm_p), PWM_N(pwm_n) {
    GATE_DRIVER_ENABLE.write(DISABLE);
    PWM_P.period_us(PERIOD);
    PWM_N.period_us(PERIOD);
    PWM_P.pulsewidth_us(0);
    PWM_N.pulsewidth_us(0);
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
        if(v >= PERIOD * PWM_LIMIT) v = PERIOD * PWM_LIMIT;
        PWM_P.pulsewidth_us(v);
    }
    else if(v < 0){
        if(v <= -PERIOD * PWM_LIMIT) v = -PERIOD * PWM_LIMIT;
        PWM_N.pulsewidth_us(-v);
    }
    else {
        PWM_P.pulsewidth_us(0);
        PWM_N.pulsewidth_us(0);
    }
}