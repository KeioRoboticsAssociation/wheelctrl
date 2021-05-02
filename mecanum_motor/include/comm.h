#ifndef __COMM_H
#define __COMM_H

class Status;
class Mbedserial;

class Comm {
public:
    Comm();
    void process(Status &status, Mbedserial &Ms);
private:
    float data;
};

#endif