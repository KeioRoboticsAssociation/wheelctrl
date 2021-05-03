#ifndef __PID_H
#define __PID_H

#include "mbed.h"
#include "param.h"

class PID {
public:
    PID(const float &a, const float &b, const float &c);
    float calc_speed_command(float target_value, float current_value);
    float calc_position_command(float target_value, float current_value);

private:
    float _a, _b, _c;
    float p, i, d;
    float Kp, Ki, Kd;
    float d_speed[2]; // [0]が1ステップ前、[1]が最新
    float integral;
    float command_value;
};

#endif