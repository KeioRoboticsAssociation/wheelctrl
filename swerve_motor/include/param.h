#ifndef __PARAM_H__
#define __PARAM_H__

#define PI 3.14159265359
#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define PERIOD 1000  // PWM制御のパルス幅 μs
#define EN_RESOLUTION 360  // エンコーダーの分解能
#define WHEEL_R  0.035  // タイヤの半径 m
#define PWM_LIMIT 0.3
#define INTEGRAL_LIMIT 100  // PID制御のIの値の発散防止

#define PIN_GATE_DRIVER_ENABLE_WHEEL PB_5
#define PIN_PWM_P_WHEEL PA_10
#define PIN_PWM_N_WHEEL PA_11_ALT0

#define PIN_GATE_DRIVER_ENABLE_TABLE PB_4
#define PIN_PWM_P_TABLE PB_0_ALT0
#define PIN_PWM_N_TABLE PB_1_ALT0

#define PIN_ENA_WHEEL PA_8
#define PIN_ENB_WHEEL PA_9

#define PIN_ENA_TABLE PA_6
#define PIN_ENB_TABLE PA_7_ALT0

#define Kp_WHEEL 10.0
#define Ki_WHEEL 0.0
#define Kd_WHEEL 0.0

#define Kp_TABLE 10.0
#define Ki_TABLE 0.0
#define Kd_TABLE 0.0

#endif