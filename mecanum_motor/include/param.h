#ifndef __PARAM_H__
#define __PARAM_H__

#define PI 3.14159265359
#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define COMM_TIME_US 10000
#define PERIOD 1000  // PWM制御のパルス幅 μs
#define RESOLUTION 10  // エンコーダーの分解能
#define TIRE_R  0.1  // タイヤの半径 m
#define PWM_LIMIT 50
#define INTEGRAL_LIMIT 100  // PID制御のIの値の発散防止

#endif