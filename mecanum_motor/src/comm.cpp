#include "comm.h"

Comm::Comm(Mbedserial &Ms): _Ms(Ms)
{
  _target_value = 0;
  _current_value[0] = 0;
}
void Comm::process() {
  _target_value = _Ms.getfloat[0];
  _Ms.float_write(_current_value, 1);
}