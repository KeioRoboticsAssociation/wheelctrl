#ifndef __PID_H
#define __PID_H

#include "mbed.h"
#include "param.h"

class PID {
public:
    PID(float KP, float KI, float KD);
    float calc_speed_command(float target_value, float current_value);
    float calc_position_command(float target_value, float current_value);

private:
    float p, i, d;
    float _Kp, _Ki, _Kd;
    float d_speed[2]; // [0]が1ステップ前、[1]が最新
    float integral;
    float command_value;
};

#endif