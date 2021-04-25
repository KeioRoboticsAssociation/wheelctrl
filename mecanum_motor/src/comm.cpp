#include "mbed.h"

void Comm::process(void) {
  target_speed = Ms.getfloat[0];
  Ms.float_write(current_speed, 1);
}