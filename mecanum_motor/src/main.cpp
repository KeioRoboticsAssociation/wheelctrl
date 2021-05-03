#include <mbed.h>
#include "main.h"
#include "mbedserial.h"
#include "motordriver.h"
#include "pid.h"
#include "encoder.h"
#include "comm.h"

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
InterruptIn switch1(USER_BUTTON);
Ticker ticker1, ticker2;
Status status;

// モーター
PwmOut PWM(PA_8);
DigitalOut PHASE(PC_11);
Motor motor(PWM, PHASE, PERIOD);

// エンコーダ
InterruptIn enA(PA_9);
DigitalIn enB(PB_4);
Encoder encoder(enA, enB);

// PID
PID pid(1, 0, 0);  // Kp, Ki, Kd

// シリアル通信
Comm comm();

int main() {
  while(switch1);
  while(1) wait(1);
}
