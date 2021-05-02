#include "mbed.h"
#include "main.h"
#include "encoder.h"

Encoder::Encoder(InterruptIn &enA, DigitalIn &enB) : _enA(enA), _enB(enB)
{
    en_count = 0;
    old_en_count = 0;
    d_en_count = 0;
}

void Encoder::count(void)
{
    if (_enB == 1)
        en_count += 1;
    else
        en_count -= 1;
}

void Encoder::calc(Status &status)
{
    d_en_count = en_count - old_en_count;
    old_en_count = en_count;
    status.current_speed = d_en_count * 1000000 / SUMPLING_TIME_US / RESOLUTION * 2 * PI * TIRE_R;
}

void Encoder::startCounter(void)
{
    _enA.rise(callback(this, &Encoder::count));
}