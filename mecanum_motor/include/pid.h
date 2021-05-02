#ifndef __PID_H
#define __PID_H

#include "mbed.h"

class PID {
public:
    PID(double &a, double &b, double &c);
    void calc(void);
private:
    double _a, _b, _c;
    double p, i, d;
    double Kp, Ki, Kd;
    double d_speed[2]; // [0]が1ステップ前、[1]が最新
    double integral;
};

#endif