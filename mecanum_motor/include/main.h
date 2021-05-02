#ifndef __MAIN_H__
#define __MAIN_H__

#define PI 3.14159265359
#define SUMPLING_TIME_US 1000  // タイマー割り込み周期 kHzオーダーつまり1000が理想
#define COMM_TIME_US 10000
#define PERIOD 1000.0  // PWM制御のパルス幅 μs
#define RESOLUTION 10  // エンコーダーの分解能
#define TIRE_R  0.1  // タイヤの半径 m

class Status {
public:
    double target_speed;  // 目標値 m/s
    double current_speed;  // 現在値 m/s
    double command_v;  // 指令値 (pwm μs)
    Status() :
        target_speed(0),
        current_speed(0), 
        command_v(0) {}
};

int main();

#endif