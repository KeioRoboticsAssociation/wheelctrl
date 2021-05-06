#include "comm.h"

Comm::Comm(Mbedserial &Ms) : _Ms(Ms)
{
  target_value = 0;
  current_value = 0;
}

void Comm::startCommunication(void){
  ticker.attach_us(callback(this, &Comm::process), COMM_TIME_US);
}

void Comm::process() {
  target_value = _Ms.getfloat[0];
  _Ms.float_write(&current_value, 1);
}