#include "mbedserial.h"
#include "motordriver.h"
#include "pid.h"
#include "encoder.h"
#include "comm.h"



float target_value_table = 0;  // 目標値 deg
int current_value_table = 0;  // 現在値 deg
float command_value_table = 0;  // 指令値 (pwm μs)

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
Ticker ticker_table;

DigitalOut myled(LED1);
DigitalIn switch1(PB_7);
DigitalIn switch2(PB_6);
DigitalOut OE(PF_1, 0); // レベルシフタを有効化

// モーター

Motor motor_table(PIN_GATE_DRIVER_ENABLE_TABLE, PIN_PWM_P_TABLE, PIN_PWM_N_TABLE, PWM_LIMIT_TABLE);

// エンコーダ

Encoder encoder_table(PIN_ENA_TABLE, PIN_ENB_TABLE, EN_RESOLUTION_TABLE);

// PID

PID pid_table(Kp_TABLE, Ki_TABLE, Kd_TABLE);

// シリアル通信
Comm comm(Ms);


float transform_table_to_encoder(float value){ return value * 122.0f / 40.0f * 5.0f / PI * 180.0f;};// deg
float transform_encoder_to_table(float value) { return value / 122.0f * 40.0f / 5.0f * PI / 180.0f; }; // rad



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
  
  encoder_table.startCounter();
  
  motor_table.enableGateDriver();
  
  ticker_table.attach_us(&Timer_Interrupt_table, SUMPLING_TIME_US);
  Ms.float_attach(Comm_Interrupt);

  wait_switch2_on();
  myled = 0;
  
  motor_table.disableGateDriver();
  
  ticker_table.detach();
}
