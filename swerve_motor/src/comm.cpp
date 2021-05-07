#include "comm.h"

Comm::Comm(Mbedserial &Ms) : _Ms(Ms)
{
  _target_velocity = 0;
  _target_theta = 0;
  _current_value[0] = 0;
  _current_value[1] = 0;
}

void Comm::process() {
  _target_velocity = _Ms.getfloat[0];
  _target_theta = _Ms.getfloat[1];
  /*\
  _current_value[0] = _target_velocity;
  _current_value[1] = _target_theta;
  */
  _Ms.float_write(_current_value, 2);
}