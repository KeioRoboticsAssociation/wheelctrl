#include "mbed.h"
#include "pid.h"
#include "motordriver.h"

PID::PID(double &a, double &b, double &c) : _a(a), _b(b), _c(c) {
    Kp = _a;
    Ki = _b;
    Kd = _c;
    integral = 0;
}

void PID::calc(void) {
  d_speed[0] = d_speed[1];
  d_speed[1] = target_speed - current_speed;
  integral += (d_speed[1] + d_speed[0]) / 2.0 * (SUMPLING_TIME_US / (double)1000000);
  p = Kp * d_speed[1];
  i = Ki * integral;
  d = Kd * (d_speed[0] - d_speed[1]) / (SUMPLING_TIME_US　/　(double)1000000);
  command_v += p + i + d;
  motor.speed(command_v);
}
