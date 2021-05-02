#ifndef __ENCODER_H__
#define __ENCODER_H__

#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想

class Encoder
{
public:
    Encoder(InterruptIn &enA, DigitalIn &enB);
    void startCounter(void);
    float getEncoderValue(void) { return en_count; };
    float getCurrentSpeed(void) { return current_speed; };

private:
    InterruptIn _enA;
    DigitalIn _enB;
    Ticker ticker;

    float en_count;
    float old_en_count;
    float d_en_count;
    float current_speed;

    void count(void);
    void calc();
};

#endif