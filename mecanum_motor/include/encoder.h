#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "mbed.h"
#include "param.h"


class Encoder
{
public:
    Encoder(InterruptIn &enA, InterruptIn &enB, int resolution);
    void startCounter(void);
    void calc_speed(void);
    float getEncoderValue(void) { return en_count; };
    float getCurrentSpeed(void) { return current_speed; };

private:
    InterruptIn _enA;
    InterruptIn _enB;
    int _resolution;

    float en_count;
    float old_en_count;
    float d_en_count;
    float current_speed;

    void counterArise(void);
    void counterAfall(void);
    void counterBrise(void);
    void counterBfall(void);
};

#endif