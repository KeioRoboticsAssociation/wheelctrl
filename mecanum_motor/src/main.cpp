#include <mbed.h>
#include "motordriver.h"
#include "PID.h"
#include "controller.h"
#include "main.h"
#include "mbedserial.h"

#define SUMPLING_TIME_US 100000 //タイマー割り込み周期 kHzオーダーつまり1000が理想
#define PERIOD 1000 //モーターのMAX値
const double resolution = 10;  // エンコーダーの分解能
const double r = 10;  // タイヤの半径 / m

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
InterruptIn switch1(USER_BUTTON);
Ticker ticker1, ticker2, ticker3, ticker4;

//モーター
PwmOut PWM(PA_8);
DigitalOut PHASE(PC_11);
Motor motor(PWM, PHASE, PERIOD, true);

//エンコーダ
InterruptIn enA(PA_9);
DigitalIn enB(PB_4);

// モーターの速度
double target_speed;  // 目標値
double current_speed;  // 現在値
double next_speed;  // 指令値

//エンコーダ
int en_count = 0;
int old_en_count = 0;
int d_en_count = 0;

//PID
double p, i, d;
const double Kp = 1.0, Ki = 0, Kd = 0;
double diff[2];
static double integral = 0;

//エンコーダ
void encoder(void) {
  if(enB == 1) en_count += 1;
  else en_count -= 1;
  d_en_count = en_count - old_en_count;
  old_en_count = en_count;
  current_speed = d_en_count * 1000000 / SUMPLING_TIME_US / resolution * 2 * pi * r;
}

void limit(double &a){
  if(a >= PERIOD * 0.8) a = PERIOD * 0.8;
  else if(a　<=　-1　*　PERIOD　*　0.8) a　=　-1　*　PERIOD　*　0.8;
}

//PID
void PID(void) {
  diff[0] = diff[1];
  diff[1] = target_speed - current_speed;
  integral += (diff[1] + diff[0]) / 2.0 * (SUMPLING_TIME_US / (double)1000000);
  p = Kp * diff[1];
  i = Ki * integral;
  d = Kd * (diff[0] - diff[1]) / (SUMPLING_TIME_US　/　(double)1000000);
  v += p + i + d;  // エンコーダーの差分から車輪の差分に変換し、足す
  limit(v);
}

void motor_run() {
  motor.speed(v);
}

void receive_speed() {
  target_speed = Ms.getfloat[0];
}

int main() {
  while(switch1);
  enA.rise(encoder);
  ticker1.attach_us(&speed_calc, SUMPLING_TIME_US);
  ticker2.attach_us(&PID, SUMPLING_TIME_US);
  ticker3.attach_us(&motor_run, SUMPLING_TIME_US);
  ticker4.attach_us(&receive_speed, SUMPLING_TIME_US);
}
