#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "mbed.h"
#include "param.h"


class Encoder
{
public:
    Encoder(PinName ena, PinName enb, int en_resolution);
    void startCounter(void);
    void calc_speed(void);
    int getEncoderValue(void) { return en_count; };
    float getCurrentSpeed(void) { return current_speed; };

private:
    InterruptIn _enA;
    InterruptIn _enB;
    int en_resolution_;

    int en_count;
    float old_en_count;
    float d_en_count;
    float current_speed;

    void counterArise(void);
    void counterAfall(void);
    void counterBrise(void);
    void counterBfall(void);
};

#endif