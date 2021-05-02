#ifndef __ENCODER_H__
#define __ENCODER_H__

class Encoder {
public:
    Encoder(InterruptIn &enA, DigitalIn &enB);
    void count(void);
    void calc(Status &status);
private:
    InterruptIn _enA;
    DigitalIn _enB;
    int en_count;
    int old_en_count;
    int d_en_count;
};

#endif