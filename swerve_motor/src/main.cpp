#include "mbedserial.h"
#include "motordriver.h"
#include "pid.h"
#include "encoder.h"
#include "comm.h"

float target_value_wheel = 0;  // 目標値 m/s
float current_value_wheel = 0;  // 現在値 m/s
float command_value_wheel = 0;  // 指令値 (pwm μs)

float target_value_table = 0;  // 目標値 deg
int current_value_table = 0;  // 現在値 deg
float command_value_table = 0;  // 指令値 (pwm μs)

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
Ticker ticker_wheel;
Ticker ticker_table;

DigitalOut myled(LED1);
DigitalIn switch1(PB_7);
DigitalIn switch2(PB_6);
DigitalOut OE(PF_1, 0); // レベルシフタを有効化

// モーター
Motor motor_wheel(PIN_GATE_DRIVER_ENABLE_WHEEL, PIN_PWM_P_WHEEL, PIN_PWM_N_WHEEL, PWM_LIMIT_WHEEL);
Motor motor_table(PIN_GATE_DRIVER_ENABLE_TABLE, PIN_PWM_P_TABLE, PIN_PWM_N_TABLE, PWM_LIMIT_TABLE);

// エンコーダ
Encoder encoder_wheel(PIN_ENA_WHEEL, PIN_ENB_WHEEL, EN_RESOLUTION_WHEEL);
Encoder encoder_table(PIN_ENA_TABLE, PIN_ENB_TABLE, EN_RESOLUTION_TABLE);

// PID
PID pid_wheel(Kp_WHEEL, Ki_WHEEL, Kd_WHEEL);
PID pid_table(Kp_TABLE, Ki_TABLE, Kd_TABLE);

// シリアル通信
Comm comm(Ms);

float transform_wheel_to_encoder(float value){ return value / WHEEL_R * 60.0f / 23.0f;};// rad/s
float transform_encoder_to_wheel(float value) { return value * WHEEL_R * 23.0f / 60.0f; };// m/s

float transform_table_to_encoder(float value){ return value * 122.0f / 40.0f * 5.0f / PI * 180.0f;};// deg
float transform_encoder_to_table(float value) { return value / 122.0f * 40.0f / 5.0f * PI / 180.0f; }; // rad

void Timer_Interrupt_wheel(void){
  encoder_wheel.calc_speed();
  current_value_wheel = encoder_wheel.getCurrentSpeed();
  //pc.printf("wheel_speed: %d\n",(int)current_value_wheel);
  comm.AttachCurrentVelocity(transform_encoder_to_wheel(current_value_wheel));
  target_value_wheel = transform_wheel_to_encoder(comm.getTargetVelcity());
  command_value_wheel = pid_wheel.calc_speed_command(target_value_wheel, current_value_wheel);
  motor_wheel.speed(command_value_wheel);
}

void Timer_Interrupt_table(void){
  current_value_table = encoder_table.getEncoderValue();
  //pc.printf("en_count: %d\n", current_value_table);
  comm.AttachCurrentTheta(transform_encoder_to_table((float)current_value_table));
  target_value_table = transform_table_to_encoder(comm.getTargetTheta());
  command_value_table = pid_table.calc_position_command(target_value_table, (float)current_value_table);
  motor_table.speed(command_value_table);
}

void Comm_Interrupt(void){
  comm.process();
}

bool wait_switch1_on(){
  while (1){
    if (switch1.read() == 1){
      return true;
    }
  }
}

bool wait_switch1_off(){
  while (1){
    if (switch1.read() == 0){
      return true;
    }
  }
}

bool wait_switch2_on(){
  while (1){
    if (switch2.read() == 1){
      return true;
    }
  }
}

bool wait_switch2_off(){
  while (1){
    if (switch2.read() == 0){
      return true;
    }
  }
}

int main()
{
  myled = 0;
  wait_switch1_on();
  myled = 1;
  pc.printf("start\n");
  encoder_wheel.startCounter();
  encoder_table.startCounter();
  motor_wheel.enableGateDriver();
  motor_table.enableGateDriver();
  ticker_wheel.attach_us(&Timer_Interrupt_wheel, SUMPLING_TIME_US);
  ticker_table.attach_us(&Timer_Interrupt_table, SUMPLING_TIME_US);
  Ms.float_attach(Comm_Interrupt);

  wait_switch2_on();
  myled = 0;
  motor_wheel.disableGateDriver();
  motor_table.disableGateDriver();
  ticker_wheel.detach();
  ticker_table.detach();
}
