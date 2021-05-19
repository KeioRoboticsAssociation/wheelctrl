#ifndef __PARAM_H__
#define __PARAM_H__

#define PI 3.14159265359
#define SUMPLING_TIME_US 10000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define PERIOD 1000  // PWM制御のパルス幅 μs
#define EN_RESOLUTION_WHEEL 48  // エンコーダーの分解能(車輪)
#define EN_RESOLUTION_TABLE 360  // エンコーダーの分解能(回転テーブル)
#define WHEEL_R  0.035  // タイヤの半径 m
#define PWM_LIMIT_WHEEL 0.90
#define PWM_LIMIT_TABLE 0.30
#define INTEGRAL_LIMIT 1.0  // PID制御のIの値の発散防止

#define Kp_WHEEL 0.50
#define Ki_WHEEL 0.0
#define Kd_WHEEL 0.0

#define Kp_TABLE 0.30
#define Ki_TABLE 0.0
#define Kd_TABLE 0.0

#define STYLE_LF

#ifdef STYLE_RB

#define PIN_GATE_DRIVER_ENABLE_WHEEL PB_5
#define PIN_PWM_P_WHEEL PA_11_ALT0
#define PIN_PWM_N_WHEEL PA_10

#define PIN_GATE_DRIVER_ENABLE_TABLE PB_4
#define PIN_PWM_P_TABLE PB_0_ALT0
#define PIN_PWM_N_TABLE PB_1_ALT0

#define PIN_ENA_WHEEL PA_8
#define PIN_ENB_WHEEL PA_9

#define PIN_ENA_TABLE PA_6
#define PIN_ENB_TABLE PA_7

#endif

#ifdef STYLE_RF

#define PIN_GATE_DRIVER_ENABLE_WHEEL PB_5
#define PIN_PWM_P_WHEEL PA_11_ALT0
#define PIN_PWM_N_WHEEL PA_10

#define PIN_GATE_DRIVER_ENABLE_TABLE PB_4
#define PIN_PWM_P_TABLE PB_0_ALT0
#define PIN_PWM_N_TABLE PB_1_ALT0

#define PIN_ENA_WHEEL PA_8
#define PIN_ENB_WHEEL PA_9

#define PIN_ENA_TABLE PA_6
#define PIN_ENB_TABLE PA_7

#endif

#ifdef STYLE_LF

#define PIN_GATE_DRIVER_ENABLE_WHEEL PB_5
#define PIN_PWM_P_WHEEL PA_11_ALT0
#define PIN_PWM_N_WHEEL PA_10

#define PIN_GATE_DRIVER_ENABLE_TABLE PB_4
#define PIN_PWM_P_TABLE PB_0_ALT0
#define PIN_PWM_N_TABLE PB_1_ALT0

#define PIN_ENA_WHEEL PA_8
#define PIN_ENB_WHEEL PA_9

#define PIN_ENA_TABLE PA_6
#define PIN_ENB_TABLE PA_7

#endif

#ifdef STYLE_LB

#define PIN_GATE_DRIVER_ENABLE_WHEEL PB_5
#define PIN_PWM_P_WHEEL PA_11_ALT0
#define PIN_PWM_N_WHEEL PA_10

#define PIN_GATE_DRIVER_ENABLE_TABLE PB_4
#define PIN_PWM_P_TABLE PB_1_ALT0
#define PIN_PWM_N_TABLE PB_0_ALT0

#define PIN_ENA_WHEEL PA_8
#define PIN_ENB_WHEEL PA_9

#define PIN_ENA_TABLE PA_6
#define PIN_ENB_TABLE PA_7

#endif

#endif