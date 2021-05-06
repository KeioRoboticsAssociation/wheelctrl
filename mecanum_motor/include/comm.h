#ifndef __COMM_H
#define __COMM_H

#include "mbed.h"
#include "mbedserial.h"
#include "param.h"

class Comm {
public:
    Comm(Mbedserial &Ms);
    void process(void);
    void AttachCurrentValue(float value) { _current_value[0] = value; };
    float getTargetValue(void) { return _target_value; };
    void startCommunication(void);

private:
  Mbedserial _Ms;
  float _target_value;
  float _current_value[1];
};

#endif