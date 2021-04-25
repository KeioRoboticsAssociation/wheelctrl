#include <mbed.h>
#include "main.h"
#include "mbedserial.h"
#include "motordriver.h"
#include "pid.h"
#include "controller.h"
#include "encoder.h"
#include "comm.h"

#define PI 3.14159265359
#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define COMM_TIME_US 10000
#define PERIOD 1000  // PWM制御のパルス幅 μs
#define RESOLUTION 10  // エンコーダーの分解能
#define TIRE_R  0.1  // タイヤの半径 m

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
Encoder encoder(enA, enB);

// モーターの速度
double target_speed = 0;  // 目標値 m/s
double current_speed = 0;  // 現在値 m/s
double command_v = 0;  // 指令値 (pwm μs)

//PID
PID pid(1, 0, 0);  // Kp, Ki, Kd

Comm comm;

int main() {
  while(switch1);
  ticker1.attach_us(&pid.calc, SUMPLING_TIME_US);
  ticker2.attach_us(&comm.process, COMM_TIME_US);
  while(1) wait(1);
}
