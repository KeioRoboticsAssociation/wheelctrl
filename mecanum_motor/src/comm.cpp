#include "mbed.h"
#include "main.h"
#include "mbedserial.h"

Comm::Comm(){}

void Comm::process(Status &status, Mbedserial &Ms) {
  status.target_speed = Ms.getfloat[0];
  float data = status.current_speed;
  Ms.float_write(&data, 1);
}