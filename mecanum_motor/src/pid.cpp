#include "pid.h"
#include "param.h"

PID::PID() {
    p = i = d = 0;
    d_speed[0] = d_speed[1] = 0;
    integral = 0;
    command_value = 0;
}

float PID::calc_speed_command(float target_value, float current_value) {
  d_speed[0] = d_speed[1];
  d_speed[1] = target_value - current_value;
  integral += (d_speed[1] + d_speed[0]) / 2.0 * (SUMPLING_TIME_US / (float)1000000);
  if(integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  p = Kp * d_speed[1];
  i = Ki * integral;
  d = Kd * (d_speed[0] - d_speed[1]) / (SUMPLING_TIME_US / (float)1000000);
  command_value += p + i + d;
  return command_value;
}

float PID::calc_position_command(float target_value, float current_value) {
  d_speed[0] = d_speed[1];
  d_speed[1] = target_value - current_value;
  integral += (d_speed[1] + d_speed[0]) / 2.0 * (SUMPLING_TIME_US / (float)1000000);
  if(integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  p = Kp * d_speed[1];
  i = Ki * integral;
  d = Kd * (d_speed[0] - d_speed[1]) / (SUMPLING_TIME_US / (float)1000000);
  command_value = p + i + d;
  return command_value;
}
