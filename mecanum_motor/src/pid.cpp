#include "mbed.h"
#include "main.h"
#include "pid.h"
#include "motordriver.h"

Ticker ticker;

PID::PID(double a, double b, double c) : _a(a), _b(b), _c(c) {
    Kp = _a;
    Ki = _b;
    Kd = _c;
    integral = 0;
    ticker.attach_us(callback(this, &PID::calc));
}

void PID::calc(Status &status) {
  d_speed[0] = d_speed[1];
  d_speed[1] = status.target_speed - status.current_speed;
  integral += (d_speed[1] + d_speed[0]) / 2.0 * (SUMPLING_TIME_US / (double)1000000);
  p = Kp * d_speed[1];
  i = Ki * integral;
  d = Kd * (d_speed[0] - d_speed[1]) / (SUMPLING_TIME_US / (double)1000000);
  status.command_v += p + i + d;
}
