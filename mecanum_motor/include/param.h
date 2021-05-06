#ifndef __PARAM_H__
#define __PARAM_H__

#define PI 3.14159265359
#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define COMM_TIME_US 10000
#define PERIOD 1000  // PWM制御のパルス幅 μs
#define EN_RESOLUTION 360  // エンコーダーの分解能
#define WHEEL_R  0.035  // タイヤの半径 m
#define PWM_LIMIT 0.5
#define INTEGRAL_LIMIT 100  // PID制御のIの値の発散防止

#define PIN_GATE_DRIVER_ENABLE PB_5
#define PIN_PWM_P PA_10
#define PIN_PWM_N PA_2

#define PIN_ENA PA_0
#define PIN_ENB PA_1

#define Kp 1.0
#define Ki 0.0
#define Kd 0.0

#endif