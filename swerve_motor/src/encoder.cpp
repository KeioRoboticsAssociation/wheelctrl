#include "encoder.h"

Encoder::Encoder(PinName ena, PinName enb) : _enA(ena), _enB(enb) {
    en_count = 0;
    old_en_count = 0;
    d_en_count = 0;
    current_speed = 0;
}

void Encoder::counterArise(void)
{ // call this function when the PhaseA rises
    if (_enB.read())
        en_count--;
    else
        en_count++;
}

void Encoder::counterAfall(void)
{
    if (_enB.read())
        en_count++;
    else
        en_count--;
}

void Encoder::counterBrise(void)
{
    if (_enA.read())
        en_count++;
    else
        en_count--;
}

void Encoder::counterBfall(void)
{
    if (_enA.read())
        en_count--;
    else
        en_count++;
}

void Encoder::calc_speed()
{
    d_en_count = en_count - old_en_count;
    old_en_count = en_count;
    current_speed = d_en_count * 360 / EN_RESOLUTION / 4 * 1000000 / SUMPLING_TIME_US * 2 * PI / 360; // rad/s
}


void Encoder::startCounter(void)
{
    _enA.rise(callback(this, &Encoder::counterArise));
    _enA.fall(callback(this, &Encoder::counterAfall));
    _enB.rise(callback(this, &Encoder::counterBrise));
    _enB.fall(callback(this, &Encoder::counterBfall));
}
