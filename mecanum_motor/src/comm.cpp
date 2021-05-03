#include "comm.h"
#include <mbed.h>
#include "main.h"
#include "mbedserial.h"

Ticker ticker;

Comm::Comm(Status &status, Mbedserial &Ms) : _status(status), _Ms(Ms) {
  ticker.attach_us(callback(this, &Comm::process));
}

static void Comm::process() {
  _status.target_speed = _Ms.getfloat[0];
  data = _status.current_speed;
  _Ms.float_write(&data, 1);
}