#ifndef __COMM_H
#define __COMM_H

#include "mbed.h"
#include "mbedserial.h"
#include "param.h"

class Comm {
public:
    Comm(Mbedserial &Ms);
    void process(void);
    void AttachCurrentValue(float value) { current_value = value; };
    float getTargetValue(void) { return target_value; };
    void startCommunication(void);

private:
    Mbedserial _Ms;
    Ticker ticker;
    float target_value;
    float current_value;
};

#endif