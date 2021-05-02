#include <mbed.h>
#include "main.h"
#include "mbedserial.h"
#include "motordriver.h"
#include "pid.h"
#include "controller.h"
#include "encoder.h"
#include "comm.h"

// CONSTANT
#define PI 3.14159265359
#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define PERIOD 1000  // PWM制御のパルス幅 μs
#define RESOLUTION 10  // エンコーダーの分解能
#define TIRE_R  0.1  // タイヤの半径 m

float target_value = 0;  // 目標値 m/s
float current_value = 0;  // 現在値 m/s
float command_value = 0;  // 指令値 (pwm μs)

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
InterruptIn switch1(USER_BUTTON);
Ticker ticker;

// モーター
PwmOut PWM(PA_8);
DigitalOut PHASE(PC_11);
Motor motor(PWM, PHASE, PERIOD);

// エンコーダ
InterruptIn enA(PA_9);
DigitalIn enB(PB_4);
Encoder encoder(enA, enB);

// PID
PID pid(1.0, 0.0, 0.0); // Kp, Ki, Kd

// シリアル通信
Comm comm(Ms);

float transform_wheel_to_encoder(float value){ return value * 60.0f / 23.0f;};
float transform_encoder_to_wheel(float value) { return value * 23.0f / 60.0f; };

void Timer_Interrupt(void){
  current_value = encoder.getCurrentSpeed();
  comm.AttachCurrentValue(transform_encoder_to_wheel(current_value));
  target_value = transform_wheel_to_encoder(comm.getTargetValue());
  command_value = pid.calc_speed_command(target_value, current_value);
  motor.speed(command_value);
}

void wait_switch(){
  while (1){
    if (switch1.read() == 1)
    {
      break;
    }
  }
}

int main()
{
  wait_switch();
  encoder.startCounter();
  ticker.attach_us(&Timer_Interrupt, SUMPLING_TIME_US);
  comm.startCommunication();
  while (1)
    ;
}
