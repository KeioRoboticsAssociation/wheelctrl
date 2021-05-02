/*
sign機能(high speed forwardからhigh speed backしてモーターイカれるのを防ぎませう)をつけよう
dir 1が正転
*/
#include "mbed.h"
#include "main.h"
#include "motordriver.h"

Motor::Motor(PwmOut &pwm, DigitalOut &dir, double &period) : _pwm(pwm), _dir(dir), _period(period) {
    _pwm.period_us(_period);
    _pwm.pulsewidth_us(0);
}

void Motor::speed(double v)
{
    if (v > 0){
        if(v >= _period * 0.50) v = _period * 0.50;
        _dir = 1;
        _pwm.pulsewidth_us(v);
    }
    else if(v < 0){
        if(v <= -_period * 0.50) v = -_period * 0.50;
        _dir = 0;
        _pwm.pulsewidth_us(-v);
    }
    else _pwm.pulsewidth_us(0);
}