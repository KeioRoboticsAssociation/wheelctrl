#include "encoder.h"

Encoder::Encoder(InterruptIn &enA, DigitalIn &enB) : _enA(enA), _enB(enB)
{
    en_count = 0;
    old_en_count = 0;
    d_en_count = 0;
    current_speed = 0;
}

void Encoder::count(void)
{
    if (_enB == 1)
        en_count += 1;
    else
        en_count -= 1;
}

void Encoder::calc()
{
    d_en_count = en_count - old_en_count;
    old_en_count = en_count;
    current_speed = d_en_count * 1000000 / SUMPLING_TIME_US; // rad/s
}


void Encoder::startCounter(void)
{
    _enA.rise(callback(this, &Encoder::count));
    ticker.attach_us(callback(this, &Encoder::calc), SUMPLING_TIME_US);
}
