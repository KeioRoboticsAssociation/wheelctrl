#include "comm.h"
#include <mbed.h>
#include "main.h"
#include "mbedserial.h"

Comm::Comm(){}

static void Comm::process(Status &status, Mbedserial &Ms) {
  status.target_speed = Ms.getfloat[0];
  data = status.current_speed;
  Ms.float_write(&data, 1);
}