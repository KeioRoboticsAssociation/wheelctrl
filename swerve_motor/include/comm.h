#ifndef __COMM_H
#define __COMM_H

#include "mbed.h"
#include "mbedserial.h"
#include "param.h"

class Comm {
public:
    Comm(Mbedserial &Ms);
    void process(void);
    void AttachCurrentVelocity(float velovity) { _current_value[0] = velovity; };
    void AttachCurrentTheta(float theta) { _current_value[1] = theta; };
    float getTargetVelcity(void) { return _target_velocity; };
    float getTargetTheta(void) { return _target_theta; };

private:
    Mbedserial _Ms;
    float _target_velocity;
    float _target_theta;
    float _current_value[2];
};

#endif