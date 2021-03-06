#include "pid.h"
#include "param.h"

PID::PID(float KP, float KI, float KD) : _Kp(KP), _Ki(KI), _Kd(KD)
{
    p = i = d = 0;
    d_speed[0] = d_speed[1] = 0;
    integral = 0;
    command_value = 0;
}

float PID::calc_speed_command(float target_value, float current_value) {
  d_speed[0] = d_speed[1];
  d_speed[1] = target_value - current_value;
  integral += (d_speed[1] + d_speed[0]) / 2.0f * ((float)SUMPLING_TIME_US / (float)1000000);
  if(integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  p = _Kp * d_speed[1];
  i = _Ki * integral;
  d = _Kd * (d_speed[0] - d_speed[1]) / ((float)SUMPLING_TIME_US / (float)1000000);
  command_value += p + i + d;
  return command_value;
}

float PID::calc_position_command(float target_value, float current_value) {
  d_speed[0] = d_speed[1];
  d_speed[1] = target_value - current_value;
  integral += (d_speed[1] + d_speed[0]) / 2.0f * ((float)SUMPLING_TIME_US / (float)1000000);
  if(integral > (float)INTEGRAL_LIMIT) integral = (float)INTEGRAL_LIMIT;
  p = _Kp * d_speed[1];
  i = _Ki * integral;
  d = _Kd * (d_speed[0] - d_speed[1]) / ((float)SUMPLING_TIME_US / (float)1000000);
  command_value = p + i + d;
  return command_value;
}
