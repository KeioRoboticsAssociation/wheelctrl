#ifndef __COMM_H
#define __COMM_H

Status status;
Mbedserial Ms;

class Comm{
public:
    Comm();
    void process();
private:
    float data;
};

#endif