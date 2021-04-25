#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "mbed.h"

class Encoder {
public:
    void calc(void);
protected:
    int en_count;
    int old_en_count;
    int d_en_count;
};

#endif