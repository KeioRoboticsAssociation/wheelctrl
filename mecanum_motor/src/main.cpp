#include <mbed.h>
#include "main.h"
#include "mbedserial.h"
#include "motordriver.h"
#include "pid.h"
#include "controller.h"
#include "encoder.h"
#include "comm.h"

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
InterruptIn switch1(USER_BUTTON);
Ticker ticker1, ticker2, ticker3, ticker4;
Status status;

// モーター
PwmOut PWM(PA_8);
DigitalOut PHASE(PC_11);
Motor motor(PWM, PHASE, PERIOD, true);

// エンコーダ
InterruptIn enA(PA_9);
DigitalIn enB(PB_4);
Encoder encoder(enA, enB);

// PID
PID pid(1, 0, 0); // Kp, Ki, Kd

// シリアル通信
Comm comm();

int main()
{
  while (switch1)
    ;
  encoder.startCounter();
  ticker1.attach_us(&encoder.calc, SUMPLING_TIME_US);
  ticker2.attach_us(&pid.calc, SUMPLING_TIME_US);
  ticker3.attach_us(&motor.speed(status.command_v), SUMPLING_TIME_US);
  ticker4.attach_us(&comm.process, COMM_TIME_US);
  while (1)
    ;
}
