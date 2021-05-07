#include "mbedserial.h"
#include "motordriver.h"
#include "pid.h"
#include "encoder.h"
#include "comm.h"

float target_value = 0;  // 目標値 m/s
float current_value = 0;  // 現在値 m/s
float command_value = 0;  // 指令値 (pwm μs)

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
InterruptIn switch1(USER_BUTTON);
Ticker ticker;

DigitalOut myled(LED1);

// モーター
Motor motor(PIN_GATE_DRIVER_ENABLE, PIN_PWM_P, PIN_PWM_N);

// エンコーダ
Encoder encoder(PIN_ENA, PIN_ENB);

// PID
PID pid;

// シリアル通信
Comm comm(Ms);

float transform_wheel_to_encoder(float value){ return value / WHEEL_R * 60.0f / 23.0f;};// rad/s
float transform_encoder_to_wheel(float value) { return value * WHEEL_R * 23.0f / 60.0f; };// m/s

void Timer_Interrupt(void){
  encoder.calc_speed();
  current_value = encoder.getCurrentSpeed();
  comm.AttachCurrentValue(transform_encoder_to_wheel(current_value));
  target_value = transform_wheel_to_encoder(comm.getTargetValue());
  command_value = pid.calc_speed_command(target_value, current_value);
  motor.speed(command_value);
}

void Comm_Interrupt(void){
  comm.process();
}

bool wait_switch(){
  while (1){
    if (switch1.read() == 0){
      return true;
    }
  }
}

int main()
{
  myled = 0;
  wait_switch();
  myled = 1;
  pc.printf("start\n");
  encoder.startCounter();
  motor.enableGateDriver();
  ticker.attach_us(&Timer_Interrupt, SUMPLING_TIME_US);
  Ms.float_attach(Comm_Interrupt);
  while (!wait_switch())
    ;
}
