#ifndef __PARAM_H__
#define __PARAM_H__

#define TR
#define STYLE_LB

#define PI 3.14159265359
#define SUMPLING_TIME_US 10000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define PERIOD 1000  // PWM制御のパルス幅 μs




#define EN_RESOLUTION_TABLE 360  // エンコーダーの分解能(回転テーブル)

#define PWM_LIMIT_TABLE 0.90
#define INTEGRAL_LIMIT 1.0  // PID制御のIの値の発散防止


#define Kp_TABLE 0.35
#define Ki_TABLE 0.0
#define Kd_TABLE 0.0





#define PIN_GATE_DRIVER_ENABLE_TABLE PB_5
#define PIN_PWM_P_TABLE PA_10
#define PIN_PWM_N_TABLE PA_11_ALTO



#define PIN_ENA_TABLE PA_8
#define PIN_ENB_TABLE PA_9

#endif







