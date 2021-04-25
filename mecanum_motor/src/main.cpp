#include <mbed.h>
#include "motordriver.h"
#include "PID.h"
#include "controller.h"
#include "main.h"
#include "mbedserial.h"

#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define COMM_TIME_US 10000
#define PERIOD 1000  // PWM制御のパルス幅 μs

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
InterruptIn switch1(USER_BUTTON);
Ticker ticker1, ticker2;

//モーター
PwmOut PWM(PA_8);
DigitalOut PHASE(PC_11);
Motor motor(PWM, PHASE, PERIOD, true);

//エンコーダ
InterruptIn enA(PA_9);
DigitalIn enB(PB_4);

double resolution = 10;  // エンコーダーの分解能
double r = 0.1;  // タイヤの半径 m

// モーターの速度
double target_speed = 0;  // 目標値 m/s
double current_speed = 0;  // 現在値 m/s
double command_v = 0;  // 指令値 (pwm μs)

//エンコーダ
int en_count = 0;
int old_en_count = 0;
int d_en_count = 0;

//PID
double p, i, d;
double Kp = 1, Ki = 0, Kd = 0;
double d_speed[2]; // [0]が1ステップ前、[1]が最新
double integral = 0;

//エンコーダ
void encoder(void) {
  if(enB == 1) en_count += 1;
  else en_count -= 1;
  d_en_count = en_count - old_en_count;
  old_en_count = en_count;
  current_speed = d_en_count * 1000000 / SUMPLING_TIME_US / resolution * 2 * pi * r;
}

//PID
void PID(void) {
  d_speed[0] = d_speed[1];
  d_speed[1] = target_speed - current_speed;
  integral += (d_speed[1] + d_speed[0]) / 2.0 * (SUMPLING_TIME_US / (double)1000000);
  p = Kp * d_speed[1];
  i = Ki * integral;
  d = Kd * (d_speed[0] - d_speed[1]) / (SUMPLING_TIME_US　/　(double)1000000);
  command_v += p + i + d;
  motor.speed(command_v);
}

void COMM(void) {
  target_speed = Ms.getfloat[0];
  Ms.float_write(current_speed, 1);
}

int main() {
  while(switch1);
  enA.rise(encoder);
  ticker1.attach_us(&PID, SUMPLING_TIME_US);
  ticker2.attach_us(&COMM, COMM_TIME_US);
  while(1) wait(1);
}
